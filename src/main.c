#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

/* Custom LBS UUID definitions */
static struct bt_uuid_128 lbs_uuid = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x00001523, 0x1212, 0xefde, 0x1523, 0x785feabcd123)
);

static struct bt_uuid_128 lbs_char_uuid = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x00001524, 0x1212, 0xefde, 0x1523, 0x785feabcd123)
);

static char rx_buffer[64];
static struct bt_conn *current_conn;
static bool notifications_enabled;

/* FIX: Compact 10-byte data payload structure definition */
struct __attribute__((packed)) sensor_payload_t {
	int16_t x;     // Accel X (scaled by 10)
	int16_t y;     // Accel Y (scaled by 10)
	int16_t z;     // Accel Z (scaled by 10)
	int16_t press; // Pressure offset (scaled by 10)
	int16_t temp;  // Temperature (scaled by 10)
};

static struct sensor_payload_t tx_data;

static void lbs_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	notifications_enabled = (value == BT_GATT_CCC_NOTIFY);
	printk("Notifications %s\n", notifications_enabled ? "enabled" : "disabled");
}

static ssize_t write_lbs_char(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			     const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	if (offset + len > sizeof(rx_buffer) - 1) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}
	memcpy(rx_buffer + offset, buf, len);
	rx_buffer[offset + len] = '\0';
	return len;
}

static ssize_t read_lbs_char(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			    void *buf, uint16_t len, uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset, &tx_data, sizeof(tx_data));
}

BT_GATT_SERVICE_DEFINE(lbs_svc,
	BT_GATT_PRIMARY_SERVICE(&lbs_uuid.uuid),
	BT_GATT_CHARACTERISTIC(&lbs_char_uuid.uuid,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
			       read_lbs_char, write_lbs_char, &tx_data),
	BT_GATT_CCC(lbs_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, 
		0x23, 0xd1, 0xbc, 0xea, 0x5f, 0x78, 0x23, 0x15, 
		0xde, 0xef, 0x12, 0x12, 0x23, 0x15, 0x00, 0x00),
};

static const struct bt_data sd[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

static void connected(struct bt_conn *conn, uint8_t err) {
	if (!err) {
		current_conn = bt_conn_ref(conn);
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason) {
	if (current_conn) {
		bt_conn_unref(current_conn);
		current_conn = NULL;
	}
	notifications_enabled = false;
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

int send_sensor_notification(const void *data, uint16_t len)
{
	if (!current_conn || !notifications_enabled) {
		return -EACCES;
	}
	return bt_gatt_notify(current_conn, &lbs_svc.attrs[1], data, len);
}

int main(void)
{
	k_sleep(K_MSEC(500));
	const struct device *const mpu6050 = DEVICE_DT_GET_ONE(invensense_mpu6050);
	const struct device *const dps310  = DEVICE_DT_GET_ONE(infineon_dps310);

	if (bt_enable(NULL) || !device_is_ready(mpu6050) || !device_is_ready(dps310)) {
		return 0;
	}

	struct bt_le_adv_param adv_param = {
		.options = BT_LE_ADV_OPT_CONN,
		.interval_min = BT_GAP_ADV_FAST_INT_MIN_2,
		.interval_max = BT_GAP_ADV_FAST_INT_MAX_2,
	};
	bt_le_adv_start(&adv_param, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));

	while (1) {
		k_sleep(K_MSEC(250));

		if (notifications_enabled) {
			struct sensor_value accel_x = {0}, accel_y = {0}, accel_z = {0};
			struct sensor_value pressure = {0}, temperature = {0};

			if (sensor_sample_fetch(mpu6050) == 0) {
				sensor_channel_get(mpu6050, SENSOR_CHAN_ACCEL_X, &accel_x);
				sensor_channel_get(mpu6050, SENSOR_CHAN_ACCEL_Y, &accel_y);
				sensor_channel_get(mpu6050, SENSOR_CHAN_ACCEL_Z, &accel_z);
			}
			if (sensor_sample_fetch(dps310) == 0) {
				sensor_channel_get(dps310, SENSOR_CHAN_PRESS, &pressure);
				sensor_channel_get(dps310, SENSOR_CHAN_AMBIENT_TEMP, &temperature);
			}

			// Pack bits securely to stay below 20-byte BLE MTU bounds
			tx_data.x = (int16_t)(sensor_value_to_double(&accel_x) * 10.0);
			tx_data.y = (int16_t)(sensor_value_to_double(&accel_y) * 10.0);
			tx_data.z = (int16_t)(sensor_value_to_double(&accel_z) * 10.0);
			// Subtract standard sea-level baseline pressure (101.3 kPa) to fit int16 range safely
			tx_data.press = (int16_t)((sensor_value_to_double(&pressure) - 101.3) * 100.0);
			tx_data.temp = (int16_t)(sensor_value_to_double(&temperature) * 10.0);

			int err = send_sensor_notification(&tx_data, sizeof(tx_data));
			if (!err) {
				printk("Binary Frame Sent Successfully\n");
			}
		}
	}
	return 0;
}
