// SPDX-License-Identifier: ISC
/* Copyright (C) 2020 MediaTek Inc. */

#include <linux/etherdevice.h>
#include <linux/platform_device.h>
#include <linux/pci.h>
#include <linux/module.h>
#include "mt7915.h"
#include "mcu.h"

// defined by Yao-Wen Liu
#define MAX_CLIENTS 5
struct station_table{
	// [MCS][NSS][GI]
	unsigned long success[10][2][2];
	unsigned long attempt[10][2][2];
	unsigned long long int data_len;
	bool initialized;
};
struct station_table stations[MAX_CLIENTS];
u8 sta_addr[MAX_CLIENTS][ETH_ALEN];
bool initialized = false;
s64 previous_sec = 0;
long previous_nsec = 0;

static bool mt7915_dev_running(struct mt7915_dev *dev)
{
	struct mt7915_phy *phy;

	if (test_bit(MT76_STATE_RUNNING, &dev->mphy.state))
		return true;

	phy = mt7915_ext_phy(dev);

	return phy && test_bit(MT76_STATE_RUNNING, &phy->mt76->state);
}

static int mt7915_start(struct ieee80211_hw *hw)
{
	struct mt7915_dev *dev = mt7915_hw_dev(hw);
	struct mt7915_phy *phy = mt7915_hw_phy(hw);
	bool running;
	int ret;

	flush_work(&dev->init_work);

	mutex_lock(&dev->mt76.mutex);

	running = mt7915_dev_running(dev);

	if (!running) {
		ret = mt7915_mcu_set_pm(dev, 0, 0);
		if (ret)
			goto out;

		ret = mt7915_mcu_set_mac(dev, 0, true, true);
		if (ret)
			goto out;

		ret = mt7915_mcu_set_scs(dev, 0, true);
		if (ret)
			goto out;

		mt7915_mac_enable_nf(dev, 0);
	}

	if (phy != &dev->phy) {
		ret = mt7915_mcu_set_pm(dev, 1, 0);
		if (ret)
			goto out;

		ret = mt7915_mcu_set_mac(dev, 1, true, true);
		if (ret)
			goto out;

		ret = mt7915_mcu_set_scs(dev, 1, true);
		if (ret)
			goto out;

		mt7915_mac_enable_nf(dev, 1);
	}

	ret = mt7915_mcu_set_rts_thresh(phy, 0x92b);
	if (ret)
		goto out;

	ret = mt7915_mcu_set_sku_en(phy, true);
	if (ret)
		goto out;

	ret = mt7915_mcu_set_chan_info(phy, MCU_EXT_CMD(SET_RX_PATH));
	if (ret)
		goto out;

	set_bit(MT76_STATE_RUNNING, &phy->mt76->state);

	if (!mt76_testmode_enabled(phy->mt76))
		ieee80211_queue_delayed_work(hw, &phy->mt76->mac_work,
					     MT7915_WATCHDOG_TIME);

	if (!running)
		mt7915_mac_reset_counters(phy);

out:
	mutex_unlock(&dev->mt76.mutex);

	return ret;
}

static void mt7915_stop(struct ieee80211_hw *hw)
{
	struct mt7915_dev *dev = mt7915_hw_dev(hw);
	struct mt7915_phy *phy = mt7915_hw_phy(hw);

	cancel_delayed_work_sync(&phy->mt76->mac_work);

	mutex_lock(&dev->mt76.mutex);

	mt76_testmode_reset(phy->mt76, true);

	clear_bit(MT76_STATE_RUNNING, &phy->mt76->state);

	if (phy != &dev->phy) {
		mt7915_mcu_set_pm(dev, 1, 1);
		mt7915_mcu_set_mac(dev, 1, false, false);
	}

	if (!mt7915_dev_running(dev)) {
		mt7915_mcu_set_pm(dev, 0, 1);
		mt7915_mcu_set_mac(dev, 0, false, false);
	}

	mutex_unlock(&dev->mt76.mutex);
}

static inline int get_free_idx(u32 mask, u8 start, u8 end)
{
	return ffs(~mask & GENMASK(end, start));
}

static int get_omac_idx(enum nl80211_iftype type, u64 mask)
{
	int i;

	switch (type) {
	case NL80211_IFTYPE_MESH_POINT:
	case NL80211_IFTYPE_ADHOC:
	case NL80211_IFTYPE_STATION:
		/* prefer hw bssid slot 1-3 */
		i = get_free_idx(mask, HW_BSSID_1, HW_BSSID_3);
		if (i)
			return i - 1;

		if (type != NL80211_IFTYPE_STATION)
			break;

		/* next, try to find a free repeater entry for the sta */
		i = get_free_idx(mask >> REPEATER_BSSID_START, 0,
				 REPEATER_BSSID_MAX - REPEATER_BSSID_START);
		if (i)
			return i + 32 - 1;

		i = get_free_idx(mask, EXT_BSSID_1, EXT_BSSID_MAX);
		if (i)
			return i - 1;

		if (~mask & BIT(HW_BSSID_0))
			return HW_BSSID_0;

		break;
	case NL80211_IFTYPE_MONITOR:
	case NL80211_IFTYPE_AP:
		/* ap uses hw bssid 0 and ext bssid */
		if (~mask & BIT(HW_BSSID_0))
			return HW_BSSID_0;

		i = get_free_idx(mask, EXT_BSSID_1, EXT_BSSID_MAX);
		if (i)
			return i - 1;

		break;
	default:
		WARN_ON(1);
		break;
	}

	return -1;
}

static int mt7915_add_interface(struct ieee80211_hw *hw,
				struct ieee80211_vif *vif)
{
	struct mt7915_vif *mvif = (struct mt7915_vif *)vif->drv_priv;
	struct mt7915_dev *dev = mt7915_hw_dev(hw);
	struct mt7915_phy *phy = mt7915_hw_phy(hw);
	struct mt76_txq *mtxq;
	bool ext_phy = phy != &dev->phy;
	int idx, ret = 0;

	mutex_lock(&dev->mt76.mutex);

	mt76_testmode_reset(phy->mt76, true);

	if (vif->type == NL80211_IFTYPE_MONITOR &&
	    is_zero_ether_addr(vif->addr))
		phy->monitor_vif = vif;

	mvif->idx = ffs(~dev->mt76.vif_mask) - 1;
	if (mvif->idx >= MT7915_MAX_INTERFACES) {
		ret = -ENOSPC;
		goto out;
	}

	idx = get_omac_idx(vif->type, phy->omac_mask);
	if (idx < 0) {
		ret = -ENOSPC;
		goto out;
	}
	mvif->omac_idx = idx;
	mvif->phy = phy;
	mvif->band_idx = ext_phy;

	if (ext_phy)
		mvif->wmm_idx = ext_phy * (MT7915_MAX_WMM_SETS / 2) +
				mvif->idx % (MT7915_MAX_WMM_SETS / 2);
	else
		mvif->wmm_idx = mvif->idx % MT7915_MAX_WMM_SETS;

	ret = mt7915_mcu_add_dev_info(phy, vif, true);
	if (ret)
		goto out;

	dev->mt76.vif_mask |= BIT(mvif->idx);
	phy->omac_mask |= BIT_ULL(mvif->omac_idx);

	idx = MT7915_WTBL_RESERVED - mvif->idx;

	INIT_LIST_HEAD(&mvif->sta.rc_list);
	INIT_LIST_HEAD(&mvif->sta.stats_list);
	INIT_LIST_HEAD(&mvif->sta.poll_list);
	mvif->sta.wcid.idx = idx;
	mvif->sta.wcid.ext_phy = mvif->band_idx;
	mvif->sta.wcid.hw_key_idx = -1;
	mvif->sta.wcid.tx_info |= MT_WCID_TX_INFO_SET;
	mt7915_mac_wtbl_update(dev, idx,
			       MT_WTBL_UPDATE_ADM_COUNT_CLEAR);

	rcu_assign_pointer(dev->mt76.wcid[idx], &mvif->sta.wcid);
	if (vif->txq) {
		mtxq = (struct mt76_txq *)vif->txq->drv_priv;
		mtxq->wcid = &mvif->sta.wcid;
	}

	if (vif->type != NL80211_IFTYPE_AP &&
	    (!mvif->omac_idx || mvif->omac_idx > 3))
		vif->offload_flags = 0;
	vif->offload_flags |= IEEE80211_OFFLOAD_ENCAP_4ADDR;

out:
	mutex_unlock(&dev->mt76.mutex);

	return ret;
}

static void mt7915_remove_interface(struct ieee80211_hw *hw,
				    struct ieee80211_vif *vif)
{
	struct mt7915_vif *mvif = (struct mt7915_vif *)vif->drv_priv;
	struct mt7915_sta *msta = &mvif->sta;
	struct mt7915_dev *dev = mt7915_hw_dev(hw);
	struct mt7915_phy *phy = mt7915_hw_phy(hw);
	int idx = msta->wcid.idx;

	mt7915_mcu_add_bss_info(phy, vif, false);
	mt7915_mcu_add_sta(dev, vif, NULL, false);

	mutex_lock(&dev->mt76.mutex);
	mt76_testmode_reset(phy->mt76, true);
	mutex_unlock(&dev->mt76.mutex);

	if (vif == phy->monitor_vif)
		phy->monitor_vif = NULL;

	mt7915_mcu_add_dev_info(phy, vif, false);

	rcu_assign_pointer(dev->mt76.wcid[idx], NULL);

	mutex_lock(&dev->mt76.mutex);
	dev->mt76.vif_mask &= ~BIT(mvif->idx);
	phy->omac_mask &= ~BIT_ULL(mvif->omac_idx);
	mutex_unlock(&dev->mt76.mutex);

	spin_lock_bh(&dev->sta_poll_lock);
	if (!list_empty(&msta->poll_list))
		list_del_init(&msta->poll_list);
	spin_unlock_bh(&dev->sta_poll_lock);
}

static void mt7915_init_dfs_state(struct mt7915_phy *phy)
{
	struct mt76_phy *mphy = phy->mt76;
	struct ieee80211_hw *hw = mphy->hw;
	struct cfg80211_chan_def *chandef = &hw->conf.chandef;

	if (hw->conf.flags & IEEE80211_CONF_OFFCHANNEL)
		return;

	if (!(chandef->chan->flags & IEEE80211_CHAN_RADAR))
		return;

	if (mphy->chandef.chan->center_freq == chandef->chan->center_freq &&
	    mphy->chandef.width == chandef->width)
		return;

	phy->dfs_state = -1;
}

int mt7915_set_channel(struct mt7915_phy *phy)
{
	struct mt7915_dev *dev = phy->dev;
	int ret;

	cancel_delayed_work_sync(&phy->mt76->mac_work);

	mutex_lock(&dev->mt76.mutex);
	set_bit(MT76_RESET, &phy->mt76->state);

	mt7915_init_dfs_state(phy);
	mt76_set_channel(phy->mt76);

	ret = mt7915_mcu_set_chan_info(phy, MCU_EXT_CMD(CHANNEL_SWITCH));
	if (ret)
		goto out;

	mt7915_mac_set_timing(phy);
	ret = mt7915_dfs_init_radar_detector(phy);
	mt7915_mac_cca_stats_reset(phy);

	mt7915_mac_reset_counters(phy);
	phy->noise = 0;

out:
	clear_bit(MT76_RESET, &phy->mt76->state);
	mutex_unlock(&dev->mt76.mutex);

	mt76_txq_schedule_all(phy->mt76);

	if (!mt76_testmode_enabled(phy->mt76))
		ieee80211_queue_delayed_work(phy->mt76->hw,
					     &phy->mt76->mac_work,
					     MT7915_WATCHDOG_TIME);

	return ret;
}

static int mt7915_set_key(struct ieee80211_hw *hw, enum set_key_cmd cmd,
			  struct ieee80211_vif *vif, struct ieee80211_sta *sta,
			  struct ieee80211_key_conf *key)
{
	struct mt7915_dev *dev = mt7915_hw_dev(hw);
	struct mt7915_vif *mvif = (struct mt7915_vif *)vif->drv_priv;
	struct mt7915_sta *msta = sta ? (struct mt7915_sta *)sta->drv_priv :
				  &mvif->sta;
	struct mt76_wcid *wcid = &msta->wcid;
	u8 *wcid_keyidx = &wcid->hw_key_idx;
	int idx = key->keyidx;
	int err = 0;

	/* The hardware does not support per-STA RX GTK, fallback
	 * to software mode for these.
	 */
	if ((vif->type == NL80211_IFTYPE_ADHOC ||
	     vif->type == NL80211_IFTYPE_MESH_POINT) &&
	    (key->cipher == WLAN_CIPHER_SUITE_TKIP ||
	     key->cipher == WLAN_CIPHER_SUITE_CCMP) &&
	    !(key->flags & IEEE80211_KEY_FLAG_PAIRWISE))
		return -EOPNOTSUPP;

	/* fall back to sw encryption for unsupported ciphers */
	switch (key->cipher) {
	case WLAN_CIPHER_SUITE_AES_CMAC:
		wcid_keyidx = &wcid->hw_key_idx2;
		key->flags |= IEEE80211_KEY_FLAG_GENERATE_MMIE;
		break;
	case WLAN_CIPHER_SUITE_TKIP:
	case WLAN_CIPHER_SUITE_CCMP:
	case WLAN_CIPHER_SUITE_CCMP_256:
	case WLAN_CIPHER_SUITE_GCMP:
	case WLAN_CIPHER_SUITE_GCMP_256:
	case WLAN_CIPHER_SUITE_SMS4:
		break;
	case WLAN_CIPHER_SUITE_WEP40:
	case WLAN_CIPHER_SUITE_WEP104:
	default:
		return -EOPNOTSUPP;
	}

	mutex_lock(&dev->mt76.mutex);

	if (cmd == SET_KEY)
		*wcid_keyidx = idx;
	else if (idx == *wcid_keyidx)
		*wcid_keyidx = -1;
	else
		goto out;

	mt76_wcid_key_setup(&dev->mt76, wcid,
			    cmd == SET_KEY ? key : NULL);

	err = mt7915_mcu_add_key(dev, vif, msta, key, cmd);

out:
	mutex_unlock(&dev->mt76.mutex);

	return err;
}

static int mt7915_config(struct ieee80211_hw *hw, u32 changed)
{
	struct mt7915_dev *dev = mt7915_hw_dev(hw);
	struct mt7915_phy *phy = mt7915_hw_phy(hw);
	bool band = phy != &dev->phy;
	int ret;

	if (changed & IEEE80211_CONF_CHANGE_CHANNEL) {
#ifdef CONFIG_NL80211_TESTMODE
		if (phy->mt76->test.state != MT76_TM_STATE_OFF) {
			mutex_lock(&dev->mt76.mutex);
			mt76_testmode_reset(phy->mt76, false);
			mutex_unlock(&dev->mt76.mutex);
		}
#endif
		ieee80211_stop_queues(hw);
		ret = mt7915_set_channel(phy);
		if (ret)
			return ret;
		ieee80211_wake_queues(hw);
	}

	if (changed & IEEE80211_CONF_CHANGE_POWER) {
		ret = mt7915_mcu_set_sku(phy);
		if (ret)
			return ret;
	}

	mutex_lock(&dev->mt76.mutex);

	if (changed & IEEE80211_CONF_CHANGE_MONITOR) {
		bool enabled = !!(hw->conf.flags & IEEE80211_CONF_MONITOR);

		if (!enabled)
			phy->rxfilter |= MT_WF_RFCR_DROP_OTHER_UC;
		else
			phy->rxfilter &= ~MT_WF_RFCR_DROP_OTHER_UC;

		mt76_rmw_field(dev, MT_DMA_DCR0(band), MT_DMA_DCR0_RXD_G5_EN,
			       enabled);
		mt76_testmode_reset(phy->mt76, true);
		mt76_wr(dev, MT_WF_RFCR(band), phy->rxfilter);
	}

	mutex_unlock(&dev->mt76.mutex);

	return 0;
}

static int
mt7915_conf_tx(struct ieee80211_hw *hw, struct ieee80211_vif *vif, u16 queue,
	       const struct ieee80211_tx_queue_params *params)
{
	struct mt7915_dev *dev = mt7915_hw_dev(hw);
	struct mt7915_vif *mvif = (struct mt7915_vif *)vif->drv_priv;

	/* no need to update right away, we'll get BSS_CHANGED_QOS */
	queue = mt7915_lmac_mapping(dev, queue);
	mvif->queue_params[queue] = *params;

	return 0;
}

static void mt7915_configure_filter(struct ieee80211_hw *hw,
				    unsigned int changed_flags,
				    unsigned int *total_flags,
				    u64 multicast)
{
	struct mt7915_dev *dev = mt7915_hw_dev(hw);
	struct mt7915_phy *phy = mt7915_hw_phy(hw);
	bool band = phy != &dev->phy;
	u32 ctl_flags = MT_WF_RFCR1_DROP_ACK |
			MT_WF_RFCR1_DROP_BF_POLL |
			MT_WF_RFCR1_DROP_BA |
			MT_WF_RFCR1_DROP_CFEND |
			MT_WF_RFCR1_DROP_CFACK;
	u32 flags = 0;

#define MT76_FILTER(_flag, _hw) do {					\
		flags |= *total_flags & FIF_##_flag;			\
		phy->rxfilter &= ~(_hw);				\
		phy->rxfilter |= !(flags & FIF_##_flag) * (_hw);	\
	} while (0)

	mutex_lock(&dev->mt76.mutex);

	phy->rxfilter &= ~(MT_WF_RFCR_DROP_OTHER_BSS |
			   MT_WF_RFCR_DROP_OTHER_BEACON |
			   MT_WF_RFCR_DROP_FRAME_REPORT |
			   MT_WF_RFCR_DROP_PROBEREQ |
			   MT_WF_RFCR_DROP_MCAST_FILTERED |
			   MT_WF_RFCR_DROP_MCAST |
			   MT_WF_RFCR_DROP_BCAST |
			   MT_WF_RFCR_DROP_DUPLICATE |
			   MT_WF_RFCR_DROP_A2_BSSID |
			   MT_WF_RFCR_DROP_UNWANTED_CTL |
			   MT_WF_RFCR_DROP_STBC_MULTI);

	MT76_FILTER(OTHER_BSS, MT_WF_RFCR_DROP_OTHER_TIM |
			       MT_WF_RFCR_DROP_A3_MAC |
			       MT_WF_RFCR_DROP_A3_BSSID);

	MT76_FILTER(FCSFAIL, MT_WF_RFCR_DROP_FCSFAIL);

	MT76_FILTER(CONTROL, MT_WF_RFCR_DROP_CTS |
			     MT_WF_RFCR_DROP_RTS |
			     MT_WF_RFCR_DROP_CTL_RSV |
			     MT_WF_RFCR_DROP_NDPA);

	*total_flags = flags;
	mt76_wr(dev, MT_WF_RFCR(band), phy->rxfilter);

	if (*total_flags & FIF_CONTROL)
		mt76_clear(dev, MT_WF_RFCR1(band), ctl_flags);
	else
		mt76_set(dev, MT_WF_RFCR1(band), ctl_flags);

	mutex_unlock(&dev->mt76.mutex);
}

static void mt7915_bss_info_changed(struct ieee80211_hw *hw,
				    struct ieee80211_vif *vif,
				    struct ieee80211_bss_conf *info,
				    u32 changed)
{
	struct mt7915_phy *phy = mt7915_hw_phy(hw);
	struct mt7915_dev *dev = mt7915_hw_dev(hw);

	mutex_lock(&dev->mt76.mutex);

	/*
	 * station mode uses BSSID to map the wlan entry to a peer,
	 * and then peer references bss_info_rfch to set bandwidth cap.
	 */
	if (changed & BSS_CHANGED_BSSID &&
	    vif->type == NL80211_IFTYPE_STATION) {
		bool join = !is_zero_ether_addr(info->bssid);

		mt7915_mcu_add_bss_info(phy, vif, join);
		mt7915_mcu_add_sta(dev, vif, NULL, join);
	}

	if (changed & BSS_CHANGED_ASSOC) {
		mt7915_mcu_add_bss_info(phy, vif, info->assoc);
		mt7915_mcu_add_obss_spr(dev, vif, info->he_obss_pd.enable);
	}

	if (changed & BSS_CHANGED_ERP_SLOT) {
		int slottime = info->use_short_slot ? 9 : 20;

		if (slottime != phy->slottime) {
			phy->slottime = slottime;
			mt7915_mac_set_timing(phy);
		}
	}

	if (changed & BSS_CHANGED_BEACON_ENABLED && info->enable_beacon) {
		mt7915_mcu_add_bss_info(phy, vif, true);
		mt7915_mcu_add_sta(dev, vif, NULL, true);
	}

	/* ensure that enable txcmd_mode after bss_info */
	if (changed & (BSS_CHANGED_QOS | BSS_CHANGED_BEACON_ENABLED))
		mt7915_mcu_set_tx(dev, vif);

	if (changed & BSS_CHANGED_HE_OBSS_PD)
		mt7915_mcu_add_obss_spr(dev, vif, info->he_obss_pd.enable);

	if (changed & (BSS_CHANGED_BEACON |
		       BSS_CHANGED_BEACON_ENABLED))
		mt7915_mcu_add_beacon(hw, vif, info->enable_beacon);

	mutex_unlock(&dev->mt76.mutex);
}

static void
mt7915_channel_switch_beacon(struct ieee80211_hw *hw,
			     struct ieee80211_vif *vif,
			     struct cfg80211_chan_def *chandef)
{
	struct mt7915_dev *dev = mt7915_hw_dev(hw);

	mutex_lock(&dev->mt76.mutex);
	mt7915_mcu_add_beacon(hw, vif, true);
	mutex_unlock(&dev->mt76.mutex);
}

int mt7915_mac_sta_add(struct mt76_dev *mdev, struct ieee80211_vif *vif,
		       struct ieee80211_sta *sta)
{
	struct mt7915_dev *dev = container_of(mdev, struct mt7915_dev, mt76);
	struct mt7915_sta *msta = (struct mt7915_sta *)sta->drv_priv;
	struct mt7915_vif *mvif = (struct mt7915_vif *)vif->drv_priv;
	int ret, idx;

	idx = mt76_wcid_alloc(dev->mt76.wcid_mask, MT7915_WTBL_STA - 1);
	if (idx < 0)
		return -ENOSPC;

	INIT_LIST_HEAD(&msta->rc_list);
	INIT_LIST_HEAD(&msta->stats_list);
	INIT_LIST_HEAD(&msta->poll_list);
	msta->vif = mvif;
	msta->wcid.sta = 1;
	msta->wcid.idx = idx;
	msta->wcid.ext_phy = mvif->band_idx;
	msta->wcid.tx_info |= MT_WCID_TX_INFO_SET;
	msta->stats.jiffies = jiffies;

	mt7915_mac_wtbl_update(dev, idx,
			       MT_WTBL_UPDATE_ADM_COUNT_CLEAR);

	ret = mt7915_mcu_add_sta(dev, vif, sta, true);
	if (ret)
		return ret;

	return mt7915_mcu_add_sta_adv(dev, vif, sta, true);
}

void mt7915_mac_sta_remove(struct mt76_dev *mdev, struct ieee80211_vif *vif,
			   struct ieee80211_sta *sta)
{
	struct mt7915_dev *dev = container_of(mdev, struct mt7915_dev, mt76);
	struct mt7915_sta *msta = (struct mt7915_sta *)sta->drv_priv;

	mt7915_mcu_add_sta_adv(dev, vif, sta, false);
	mt7915_mcu_add_sta(dev, vif, sta, false);

	mt7915_mac_wtbl_update(dev, msta->wcid.idx,
			       MT_WTBL_UPDATE_ADM_COUNT_CLEAR);

	spin_lock_bh(&dev->sta_poll_lock);
	if (!list_empty(&msta->poll_list))
		list_del_init(&msta->poll_list);
	if (!list_empty(&msta->stats_list))
		list_del_init(&msta->stats_list);
	if (!list_empty(&msta->rc_list))
		list_del_init(&msta->rc_list);
	spin_unlock_bh(&dev->sta_poll_lock);
}

static void mt7915_tx(struct ieee80211_hw *hw,
		      struct ieee80211_tx_control *control,
		      struct sk_buff *skb)
{
	struct mt7915_dev *dev = mt7915_hw_dev(hw);
	struct mt76_phy *mphy = hw->priv;
	struct ieee80211_tx_info *info = IEEE80211_SKB_CB(skb);
	struct ieee80211_vif *vif = info->control.vif;
	struct mt76_wcid *wcid = &dev->mt76.global_wcid;

	if (control->sta) {
		struct mt7915_sta *sta;

		sta = (struct mt7915_sta *)control->sta->drv_priv;
		wcid = &sta->wcid;
	}

	if (vif && !control->sta) {
		struct mt7915_vif *mvif;

		mvif = (struct mt7915_vif *)vif->drv_priv;
		wcid = &mvif->sta.wcid;
	}

	mt76_tx(mphy, control->sta, wcid, skb);
}

static int mt7915_set_rts_threshold(struct ieee80211_hw *hw, u32 val)
{
	struct mt7915_dev *dev = mt7915_hw_dev(hw);
	struct mt7915_phy *phy = mt7915_hw_phy(hw);
	int ret;

	mutex_lock(&dev->mt76.mutex);
	ret = mt7915_mcu_set_rts_thresh(phy, val);
	mutex_unlock(&dev->mt76.mutex);

	return ret;
}

static int
mt7915_ampdu_action(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
		    struct ieee80211_ampdu_params *params)
{
	enum ieee80211_ampdu_mlme_action action = params->action;
	struct mt7915_dev *dev = mt7915_hw_dev(hw);
	struct ieee80211_sta *sta = params->sta;
	struct ieee80211_txq *txq = sta->txq[params->tid];
	struct mt7915_sta *msta = (struct mt7915_sta *)sta->drv_priv;
	u16 tid = params->tid;
	u16 ssn = params->ssn;
	struct mt76_txq *mtxq;
	int ret = 0;

	if (!txq)
		return -EINVAL;

	mtxq = (struct mt76_txq *)txq->drv_priv;

	mutex_lock(&dev->mt76.mutex);
	switch (action) {
	case IEEE80211_AMPDU_RX_START:
		mt76_rx_aggr_start(&dev->mt76, &msta->wcid, tid, ssn,
				   params->buf_size);
		ret = mt7915_mcu_add_rx_ba(dev, params, true);
		break;
	case IEEE80211_AMPDU_RX_STOP:
		mt76_rx_aggr_stop(&dev->mt76, &msta->wcid, tid);
		ret = mt7915_mcu_add_rx_ba(dev, params, false);
		break;
	case IEEE80211_AMPDU_TX_OPERATIONAL:
		mtxq->aggr = true;
		mtxq->send_bar = false;
		ret = mt7915_mcu_add_tx_ba(dev, params, true);
		break;
	case IEEE80211_AMPDU_TX_STOP_FLUSH:
	case IEEE80211_AMPDU_TX_STOP_FLUSH_CONT:
		mtxq->aggr = false;
		clear_bit(tid, &msta->ampdu_state);
		ret = mt7915_mcu_add_tx_ba(dev, params, false);
		break;
	case IEEE80211_AMPDU_TX_START:
		set_bit(tid, &msta->ampdu_state);
		ret = IEEE80211_AMPDU_TX_START_IMMEDIATE;
		break;
	case IEEE80211_AMPDU_TX_STOP_CONT:
		mtxq->aggr = false;
		clear_bit(tid, &msta->ampdu_state);
		ret = mt7915_mcu_add_tx_ba(dev, params, false);
		ieee80211_stop_tx_ba_cb_irqsafe(vif, sta->addr, tid);
		break;
	}
	mutex_unlock(&dev->mt76.mutex);

	return ret;
}

static int
mt7915_sta_add(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
	       struct ieee80211_sta *sta)
{
	return mt76_sta_state(hw, vif, sta, IEEE80211_STA_NOTEXIST,
			      IEEE80211_STA_NONE);
}

static int
mt7915_sta_remove(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
		  struct ieee80211_sta *sta)
{
	return mt76_sta_state(hw, vif, sta, IEEE80211_STA_NONE,
			      IEEE80211_STA_NOTEXIST);
}

static int
mt7915_get_stats(struct ieee80211_hw *hw,
		 struct ieee80211_low_level_stats *stats)
{
	struct mt7915_phy *phy = mt7915_hw_phy(hw);
	struct mt7915_dev *dev = mt7915_hw_dev(hw);
	struct mib_stats *mib = &phy->mib;

	mutex_lock(&dev->mt76.mutex);
	stats->dot11RTSSuccessCount = mib->rts_cnt;
	stats->dot11RTSFailureCount = mib->rts_retries_cnt;
	stats->dot11FCSErrorCount = mib->fcs_err_cnt;
	stats->dot11ACKFailureCount = mib->ack_fail_cnt;

	memset(mib, 0, sizeof(*mib));

	mutex_unlock(&dev->mt76.mutex);

	return 0;
}

static u64
mt7915_get_tsf(struct ieee80211_hw *hw, struct ieee80211_vif *vif)
{
	struct mt7915_vif *mvif = (struct mt7915_vif *)vif->drv_priv;
	struct mt7915_dev *dev = mt7915_hw_dev(hw);
	struct mt7915_phy *phy = mt7915_hw_phy(hw);
	bool band = phy != &dev->phy;
	union {
		u64 t64;
		u32 t32[2];
	} tsf;
	u16 n;

	mutex_lock(&dev->mt76.mutex);

	n = mvif->omac_idx > HW_BSSID_MAX ? HW_BSSID_0 : mvif->omac_idx;
	/* TSF software read */
	mt76_set(dev, MT_LPON_TCR(band, n), MT_LPON_TCR_SW_MODE);
	tsf.t32[0] = mt76_rr(dev, MT_LPON_UTTR0(band));
	tsf.t32[1] = mt76_rr(dev, MT_LPON_UTTR1(band));

	mutex_unlock(&dev->mt76.mutex);

	return tsf.t64;
}

static void
mt7915_set_tsf(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
	       u64 timestamp)
{
	struct mt7915_vif *mvif = (struct mt7915_vif *)vif->drv_priv;
	struct mt7915_dev *dev = mt7915_hw_dev(hw);
	struct mt7915_phy *phy = mt7915_hw_phy(hw);
	bool band = phy != &dev->phy;
	union {
		u64 t64;
		u32 t32[2];
	} tsf = { .t64 = timestamp, };
	u16 n;

	mutex_lock(&dev->mt76.mutex);

	n = mvif->omac_idx > HW_BSSID_MAX ? HW_BSSID_0 : mvif->omac_idx;
	mt76_wr(dev, MT_LPON_UTTR0(band), tsf.t32[0]);
	mt76_wr(dev, MT_LPON_UTTR1(band), tsf.t32[1]);
	/* TSF software overwrite */
	mt76_set(dev, MT_LPON_TCR(band, n), MT_LPON_TCR_SW_WRITE);

	mutex_unlock(&dev->mt76.mutex);
}

static void
mt7915_set_coverage_class(struct ieee80211_hw *hw, s16 coverage_class)
{
	struct mt7915_phy *phy = mt7915_hw_phy(hw);
	struct mt7915_dev *dev = phy->dev;

	mutex_lock(&dev->mt76.mutex);
	phy->coverage_class = max_t(s16, coverage_class, 0);
	mt7915_mac_set_timing(phy);
	mutex_unlock(&dev->mt76.mutex);
}

static int
mt7915_set_antenna(struct ieee80211_hw *hw, u32 tx_ant, u32 rx_ant)
{
	struct mt7915_dev *dev = mt7915_hw_dev(hw);
	struct mt7915_phy *phy = mt7915_hw_phy(hw);
	int max_nss = hweight8(hw->wiphy->available_antennas_tx);
	bool ext_phy = phy != &dev->phy;

	if (!tx_ant || tx_ant != rx_ant || ffs(tx_ant) > max_nss)
		return -EINVAL;

	if ((BIT(hweight8(tx_ant)) - 1) != tx_ant)
		tx_ant = BIT(ffs(tx_ant) - 1) - 1;

	mutex_lock(&dev->mt76.mutex);

	phy->mt76->antenna_mask = tx_ant;

	if (ext_phy) {
		if (dev->chainmask == 0xf)
			tx_ant <<= 2;
		else
			tx_ant <<= 1;
	}
	phy->mt76->chainmask = tx_ant;

	mt76_set_stream_caps(phy->mt76, true);
	mt7915_set_stream_vht_txbf_caps(phy);
	mt7915_set_stream_he_caps(phy);

	mutex_unlock(&dev->mt76.mutex);

	return 0;
}

static void mt7915_sta_statistics(struct ieee80211_hw *hw,
				  struct ieee80211_vif *vif,
				  struct ieee80211_sta *sta,
				  struct station_info *sinfo)
{
	struct mt7915_phy *phy = mt7915_hw_phy(hw);
	struct mt7915_sta *msta = (struct mt7915_sta *)sta->drv_priv;
	struct mt7915_sta_stats *stats = &msta->stats;
	struct rate_info rxrate = {};

	if (!mt7915_mcu_get_rx_rate(phy, vif, sta, &rxrate)) {
		sinfo->rxrate = rxrate;
		sinfo->filled |= BIT_ULL(NL80211_STA_INFO_RX_BITRATE);
	}

	if (!stats->tx_rate.legacy && !stats->tx_rate.flags)
		return;

	if (stats->tx_rate.legacy) {
		sinfo->txrate.legacy = stats->tx_rate.legacy;
	} else {
		sinfo->txrate.mcs = stats->tx_rate.mcs;
		sinfo->txrate.nss = stats->tx_rate.nss;
		sinfo->txrate.bw = stats->tx_rate.bw;
		sinfo->txrate.he_gi = stats->tx_rate.he_gi;
		sinfo->txrate.he_dcm = stats->tx_rate.he_dcm;
		sinfo->txrate.he_ru_alloc = stats->tx_rate.he_ru_alloc;
	}
	sinfo->txrate.flags = stats->tx_rate.flags;
	sinfo->filled |= BIT_ULL(NL80211_STA_INFO_TX_BITRATE);
}

static void
mt7915_sta_rc_update(struct ieee80211_hw *hw,
		     struct ieee80211_vif *vif,
		     struct ieee80211_sta *sta,
		     u32 changed)
{
	struct mt7915_dev *dev = mt7915_hw_dev(hw);
	struct mt7915_sta *msta = (struct mt7915_sta *)sta->drv_priv;

	spin_lock_bh(&dev->sta_poll_lock);
	msta->stats.changed |= changed;
	if (list_empty(&msta->rc_list))
		list_add_tail(&msta->rc_list, &dev->sta_rc_list);
	spin_unlock_bh(&dev->sta_poll_lock);

	ieee80211_queue_work(hw, &dev->rc_work);
}

static void mt7915_sta_set_4addr(struct ieee80211_hw *hw,
				 struct ieee80211_vif *vif,
				 struct ieee80211_sta *sta,
				 bool enabled)
{
	struct mt7915_dev *dev = mt7915_hw_dev(hw);
	struct mt7915_sta *msta = (struct mt7915_sta *)sta->drv_priv;

	if (enabled)
		set_bit(MT_WCID_FLAG_4ADDR, &msta->wcid.flags);
	else
		clear_bit(MT_WCID_FLAG_4ADDR, &msta->wcid.flags);

	mt7915_mcu_sta_update_hdr_trans(dev, vif, sta);
}

static void mt7915_sta_set_decap_offload(struct ieee80211_hw *hw,
				 struct ieee80211_vif *vif,
				 struct ieee80211_sta *sta,
				 bool enabled)
{
	struct mt7915_dev *dev = mt7915_hw_dev(hw);
	struct mt7915_sta *msta = (struct mt7915_sta *)sta->drv_priv;

	if (enabled)
		set_bit(MT_WCID_FLAG_HDR_TRANS, &msta->wcid.flags);
	else
		clear_bit(MT_WCID_FLAG_HDR_TRANS, &msta->wcid.flags);

	mt7915_mcu_sta_update_hdr_trans(dev, vif, sta);
}

static ssize_t mt7915_sta_fixed_rate_set(struct file *file,
					 const char __user *user_buf,
					 size_t count, loff_t *ppos)
{
	struct ieee80211_sta *sta = file->private_data;
	struct mt7915_sta *msta = (struct mt7915_sta *)sta->drv_priv;
	struct mt7915_dev *dev = msta->vif->phy->dev;
	struct ieee80211_vif *vif;
	struct ra_phy phy = {};
	char buf[100];
	int ret;
	u32 field;
	u8 i, gi, he_ltf;

	if (count >= sizeof(buf))
		return -EINVAL;

	if (copy_from_user(buf, user_buf, count))
		return -EFAULT;

	if (count && buf[count - 1] == '\n')
		buf[count - 1] = '\0';
	else
		buf[count] = '\0';

	/* mode - cck: 0, ofdm: 1, ht: 2, gf: 3, vht: 4, he_su: 8, he_er: 9
	 * bw - bw20: 0, bw40: 1, bw80: 2, bw160: 3
	 * nss - vht: 1~4, he: 1~4, others: ignore
	 * mcs - cck: 0~4, ofdm: 0~7, ht: 0~32, vht: 0~9, he_su: 0~11, he_er: 0~2
	 * gi - (ht/vht) lgi: 0, sgi: 1; (he) 0.8us: 0, 1.6us: 1, 3.2us: 2
	 * ldpc - off: 0, on: 1
	 * stbc - off: 0, on: 1
	 * he_ltf - 1xltf: 0, 2xltf: 1, 4xltf: 2
	 */
	if (sscanf(buf, "%hhu %hhu %hhu %hhu %hhu %hhu %hhu %hhu",
		   &phy.type, &phy.bw, &phy.nss, &phy.mcs, &gi,
		   &phy.ldpc, &phy.stbc, &he_ltf) != 8) {
		dev_warn(dev->mt76.dev,
			 "format: Mode BW NSS MCS (HE)GI LDPC STBC HE_LTF\n");
		field = RATE_PARAM_AUTO;
		goto out;
	}

	phy.ldpc = (phy.bw || phy.ldpc) * GENMASK(2, 0);
	for (i = 0; i <= phy.bw; i++) {
		phy.sgi |= gi << (i << sta->he_cap.has_he);
		phy.he_ltf |= he_ltf << (i << sta->he_cap.has_he);
	}
	field = RATE_PARAM_FIXED;

out:
	vif = container_of((void *)msta->vif, struct ieee80211_vif, drv_priv);
	ret = mt7915_mcu_set_fixed_rate_ctrl(dev, vif, sta, &phy, field);
	if (ret)
		return -EFAULT;

	return count;
}

static const struct file_operations fops_fixed_rate = {
	.write = mt7915_sta_fixed_rate_set,
	.open = simple_open,
	.owner = THIS_MODULE,
	.llseek = default_llseek,
};

static int
mt7915_sta_stats_read(struct seq_file *s, void *data)
{
	struct ieee80211_sta *sta = s->private;
	struct mt7915_sta *msta = (struct mt7915_sta *)sta->drv_priv;
	struct mt7915_sta_stats *stats = &msta->stats;
	struct rate_info *rate = &stats->tx_rate;
	static const char * const bw[] = {
		"BW20", "BW5", "BW10", "BW40",
		"BW80", "BW160", "BW_HE_RU"
	};
	
	// defined by Yao-Wen Liu
	u8 current_sta_addr[ETH_ALEN];
	unsigned long index = 0;
	unsigned long short_guard = 0;
	unsigned long current_attempt;
	unsigned long current_success;
	unsigned long current_ppdu;
	unsigned long long msec;

	memcpy(current_sta_addr, sta->addr, ETH_ALEN);	// copy station MAC address

	// initialize global variables
	if(!initialized){
		memset(sta_addr, 0, sizeof(sta_addr));
		for (index = 0; index < MAX_CLIENTS ; index ++){
			stations[index].initialized = false;
		}
		initialized = true;
	}

	// check if this is the first report
	if(previous_sec == 0 && previous_nsec == 0){
		previous_sec = stats->sec-1;
		previous_nsec = stats->nsec-1;
	}

	// find station index or add it to the array
	for (index = 0; index < MAX_CLIENTS ; index ++){
		if (current_sta_addr[0] == sta_addr[index][0] || !stations[index].initialized){
			if (!stations[index].initialized){
				memcpy(sta_addr[index], sta->addr, ETH_ALEN);
				memset(stations[index].attempt, 0, sizeof(stations[index].attempt));
				memset(stations[index].success, 0, sizeof(stations[index].success));
				stations[index].data_len = 0;
				stations[index].initialized = true;
			}
			break;
		}
	}
	
	if (!rate->legacy && !rate->flags)
		return 0;

	seq_puts(s, "Current rate - ");
	if (rate->flags & RATE_INFO_FLAGS_MCS)
		seq_puts(s, "HT ");
	else if (rate->flags & RATE_INFO_FLAGS_VHT_MCS)
		seq_puts(s, "VHT ");
	else if (rate->flags & RATE_INFO_FLAGS_HE_MCS)
		seq_puts(s, "HE ");
	else
		seq_printf(s, "Bitrate %d\n", rate->legacy);

	if (rate->flags) {
		seq_printf(s, "%s NSS%d MCS%d ",
			   bw[rate->bw], rate->nss, rate->mcs);

		if (rate->flags & RATE_INFO_FLAGS_SHORT_GI){
			seq_puts(s, "SGI ");
			short_guard = 0;
		}
		else if (rate->he_gi)
			seq_puts(s, "HE GI ");

		else
			short_guard = 1;

		if (rate->he_dcm)
			seq_puts(s, "DCM ");
	}

	// add current data to the table
	if(previous_sec != stats->sec && previous_nsec != stats->nsec){
		stations[index].attempt[rate->mcs][rate->nss][short_guard] = stations[index].attempt[rate->mcs][rate->nss][short_guard] + stats->attempts;
		stations[index].success[rate->mcs][rate->nss][short_guard] = stations[index].success[rate->mcs][rate->nss][short_guard] + stats->success;

		previous_sec = stats->sec;
		previous_nsec = stats->nsec;
	}

	current_attempt = stations[index].attempt[rate->mcs][rate->nss][short_guard];
	current_success = stations[index].success[rate->mcs][rate->nss][short_guard];
	current_ppdu = 1000 * (current_attempt - current_success) / current_attempt;

	msec = (stats->sec * 1000) + (stats->nsec / 1000000);
	
	stations[index].data_len += msta->len;
	msta->len = 0;
	
	seq_printf(s, "\nPPDU PER: %ld.%1ld%%\n", current_ppdu / 10, current_ppdu % 10);
	seq_printf(s, "\nAccumulated Attempts: %ld",stations[index].attempt[rate->mcs][rate->nss][short_guard]);
	seq_printf(s, "\nAccumulated Success: %ld\n",stations[index].success[rate->mcs][rate->nss][short_guard]);

	seq_printf(s, "\nPPDU PER: %ld.%1ld%%\n",
		   stats->per / 10, stats->per % 10);
	seq_printf(s, "msec: %llu\n", msec);
	seq_printf(s, "Len: %llu\n",stations[index].data_len);
	seq_printf(s, "Tx: %llu\n", msta->tx);
	seq_printf(s,"\n");
	return 0;
}

static int
mt7915_sta_stats_open(struct inode *inode, struct file *f)
{
	return single_open(f, mt7915_sta_stats_read, inode->i_private);
}

static const struct file_operations fops_sta_stats = {
	.open = mt7915_sta_stats_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
	.owner = THIS_MODULE,
};

void mt7915_sta_add_debugfs(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
			    struct ieee80211_sta *sta, struct dentry *dir)
{
	debugfs_create_file("fixed_rate", 0600, dir, sta, &fops_fixed_rate);
	debugfs_create_file("stats", 0400, dir, sta, &fops_sta_stats);
}

const struct ieee80211_ops mt7915_ops = {
	.tx = mt7915_tx,
	.start = mt7915_start,
	.stop = mt7915_stop,
	.add_interface = mt7915_add_interface,
	.remove_interface = mt7915_remove_interface,
	.config = mt7915_config,
	.conf_tx = mt7915_conf_tx,
	.configure_filter = mt7915_configure_filter,
	.bss_info_changed = mt7915_bss_info_changed,
	.sta_add = mt7915_sta_add,
	.sta_remove = mt7915_sta_remove,
	.sta_pre_rcu_remove = mt76_sta_pre_rcu_remove,
	.sta_rc_update = mt7915_sta_rc_update,
	.set_key = mt7915_set_key,
	.ampdu_action = mt7915_ampdu_action,
	.set_rts_threshold = mt7915_set_rts_threshold,
	.wake_tx_queue = mt76_wake_tx_queue,
	.sw_scan_start = mt76_sw_scan,
	.sw_scan_complete = mt76_sw_scan_complete,
	.release_buffered_frames = mt76_release_buffered_frames,
	.get_txpower = mt76_get_txpower,
	.channel_switch_beacon = mt7915_channel_switch_beacon,
	.get_stats = mt7915_get_stats,
	.get_tsf = mt7915_get_tsf,
	.set_tsf = mt7915_set_tsf,
	.get_survey = mt76_get_survey,
	.get_antenna = mt76_get_antenna,
	.set_antenna = mt7915_set_antenna,
	.set_coverage_class = mt7915_set_coverage_class,
	.sta_statistics = mt7915_sta_statistics,
	.sta_set_4addr = mt7915_sta_set_4addr,
	.sta_set_decap_offload = mt7915_sta_set_decap_offload,
	CFG80211_TESTMODE_CMD(mt76_testmode_cmd)
	CFG80211_TESTMODE_DUMP(mt76_testmode_dump)
//#ifdef CONFIG_MAC80211_DEBUGFS
	.sta_add_debugfs = mt7915_sta_add_debugfs,
//#endif
};
