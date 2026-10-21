#!/bin/sh
# ============================================================================
# OTA 启动期回滚守护(设备侧)
#
# 用途: 新程序一旦在启动早期就崩溃(来不及启动自身看门狗), 由本脚本兜底回滚。
#       核心机制与程序内看门狗一致: 若状态仍是"待确认(1)"且超过确认期未变为
#       "成功(2)", 就用 .bak 还原目标程序。
#
# 部署: 放到设备上(如 /usr/bin/ota_rollback.sh), 由 init/rc.local 在启动应用
#       "之后"以后台方式拉起, 例:
#           /etc/init.d/ 里 或 rc.local 中:
#               ( sleep 35; /usr/bin/ota_rollback.sh ) &
#       确认期(WAIT)要略大于应用初始化完成并 ota_mark_ok() 的时间。
#
# 手工回滚:  sh ota_rollback.sh --now
# ============================================================================

TARGET=/usr/bin/program_arm
BACKUP=/usr/bin/program_arm.bak
STATE=/var/lib/program_arm.ota_state
WAIT=35          # 确认期(秒), 需 > OTA_WATCHDOG_SEC 或应用确认时间
RUNAS=

restore() {
    if [ -f "$BACKUP" ]; then
        mv -f "$BACKUP" "$TARGET"
        echo "3" > "$STATE"     # OTA_STATE_ROLLBACK
        sync
        echo "[ota_rollback] restored $TARGET from backup"
    else
        echo "[ota_rollback] no backup, nothing to restore" >&2
    fi
}

case "$1" in
    --now)
        restore
        exit $?
        ;;
    *)
        ;;
esac

# 仅当处于"待确认"时才需要看守
[ -f "$STATE" ] || exit 0
[ "$(cat "$STATE")" = "1" ] || exit 0

echo "[ota_rollback] pending upgrade detected, watchdog $WAIT s"
sleep "$WAIT"

# 确认期内应用已标记成功则无事; 否则回滚
if [ "$(cat "$STATE" 2>/dev/null)" = "1" ]; then
    echo "[ota_rollback] not confirmed, rolling back"
    restore
else
    echo "[ota_rollback] confirmed OK, nothing to do"
fi

exit 0
