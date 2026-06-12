# Raspberry Pi Ubuntu Desktop 22.04 Setup

A step-by-step guide to installing and configuring Ubuntu Desktop 22.04 LTS on a Raspberry Pi.

---

## Requirements

- Raspberry Pi 4 B
- microSD card (64 GB)
- Computer with [Raspberry Pi Imager](https://www.raspberrypi.com/software/) installed
- HDMI cable, keyboard, and mouse
- Power supply (USB-C for RPi 4, micro-USB for RPi 3)

---

## 1. Flash Ubuntu Desktop to the microSD Card

1. Open **Raspberry Pi Imager**.
2. Click **CHOOSE DEVICE** and select **Raspberry Pi 4**.

   ![Select Device](step1_select_device.png)

3. Click **CHOOSE OS** → **Other general-purpose OS** → **Ubuntu**.
4. Select **Ubuntu Desktop 22.04.5 LTS (64-bit)** — make sure to pick **Desktop**, not Server.

   ![Choose OS](step2_choose_os.png)

5. Click **CHOOSE STORAGE** and select your microSD card.

   ![Select Storage](step3_select_storage.png)

6. Click **Next**.
7. When prompted, click **Edit Settings** to pre-configure Wi-Fi and SSH:
   - Set a **username and password**.
   - Enter your **Wi-Fi SSID and password**.
   - Set your **Wireless LAN country**.
   - Under the **Services** tab, enable **SSH** with **Use password authentication**.
8. Save settings, click **Yes** to apply, then click **WRITE** to flash the image.

   ![Write Image](step4_write_image.png)

> **Note:** Completing step 7 means you can skip manual Wi-Fi configuration after first boot.

---

## 2. First Boot

> **Important:** Connect the HDMI cable **before** powering on the Raspberry Pi. If HDMI is connected after power-on, the display output will be disabled.

1. Connect the HDMI cable to a monitor.
2. Connect a keyboard and mouse via USB.
3. Insert the flashed microSD card.
4. Connect power to turn on the Raspberry Pi.
5. Follow the on-screen setup wizard to:
   - Choose your language and keyboard layout.
   - Connect to Wi-Fi (if not pre-configured in the imager).
   - Create or confirm your user account.

---

