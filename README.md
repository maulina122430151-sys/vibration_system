**Link Website  :** https://maulina122430151-sys.github.io/vibration_system/
**Link Video    :** https://drive.google.com/file/d/1C99zHnlh4f7vWqG3dCXEZATpTMbc_hyH/view?usp=drivesdk

# 📘 **Sistem Deteksi Getaran Bed Pasien Epilepsi Berbasis ROS2 menggunakan ESP8266, SW-402, dan _Buzzer_**

## 🧭 **Deskripsi Proyek**

Proyek ini membangun sistem deteksi getaran menggunakan modul **ESP8266**, sensor **SW-402**, dan **_buzzer_** berbasis **ROS 2** dengan arsitektur **Publisher–Subscriber**. ESP8266 membaca data sensor dan mengirimkan informasi ke komputer melalui komunikasi serial. Node ROS2 kemudian memproses data dan mengaktifkan buzzer apabila terdeteksi getaran.

Sistem ini berguna untuk mendeteksi getaran pada bed/kasur pasien pengidap epilepsi. Epilepsi adalah gangguan pada sistem saraf pusat yang menyebabkan aktivitas listrik otak menjadi tidak normal sehingga memicu kejang berulang. Kejang pada epilepsi dapat berupa gerakan tubuh tak terkontrol, kaku, atau tersentak-sentak, tetapi bisa juga muncul sebagai tatapan kosong, hilang kesadaran sesaat, atau sensasi aneh sebelum serangan. Pada keadaan kejang, pasien memerlukan penanganan cepat karena gerakan tak terkontrol dapat menyebabkan cedera, gangguan pernapasan, atau memburuknya kondisi medis. 

Oleh karena itu, sistem pendeteksi getaran pada bed atau kasur pasien sangat dibutuhkan untuk mengidentifikasi aktivitas kejang sedini mungkin. Sensor getaran dapat mendeteksi pola gerakan tidak normal pada tubuh pasien dan memicu alarm bagi tenaga medis atau keluarga sehingga respons pertolongan dapat diberikan segera. Dengan deteksi dini ini, risiko komplikasi dapat ditekan dan keselamatan pasien lebih terjamin.

---
## 🤖 Anggota Kelompok
| Nama                       | NIM       |
|----------------------------|-----------|
| Maulina Adelia Putri       | 122430151 |
| Lasma Keren J. Marbun      | 122430154 |
| Elisabeth Tampubolon       | 122430104 |

---


## 📡 **Arsitektur Sistem**

### **Hardware**

* Sensor getaran SW-402
* ESP8266 (NodeMCU)
* Buzzer aktif
* Kabel USB data
* Komputer (Windows)

### **Software**

* ROS 2 (melalui environment PIXI)
* Python 3 (pyserial)
* Arduino IDE (upload firmware ESP8266)
* Colcon build system

---

# 🔧 **Cara Kerja**

1. **ESP8266 membaca sensor SW-402**

   * Mengirim data serial dengan format:

     ```
     VIB:1  → getaran terdeteksi
     VIB:0  → tidak ada getaran
     ```
2. **Node Publisher** membaca port serial → publish topic `/vibration`
3. **Node Subscriber** menerima `/vibration` → mengirim `/buzzer_cmd`
4. **Node Publisher** mengirim perintah serial ke ESP8266:

   ```
   BUZZ:ON
   BUZZ:OFF
   ```
5. ESP8266 menyalakan / mematikan buzzer.

---

# 📂 **Struktur Folder (ROS2 Workspace)**

```
ros2_ws/
└── src/
    └── vibration_system/
        ├── package.xml
        ├── setup.py
        └── vibration_system/
            ├── __init__.py
            ├── publisher_vibration.py
            └── subscriber_buzzer.py
```

---

# ⚙️ **Instalasi dan Persiapan**

## 1️⃣ Clone repositori

```
git clone https://github.com/username/vibration_system.git
cd vibration_system
```

(Ganti *username* sesuai akun Anda.)

---

## 2️⃣ Masuk ke environment PIXI

```
cd C:\pixi_ws
pixi shell
```

---

## 3️⃣ Build ROS2 Workspace

```
cd C:\pixi_ws\ros2_ws
colcon build
```

Aktifkan environment:

```
.\install\setup.ps1
```

---

# 🔌 **Upload Program ESP8266 (Arduino IDE)**

Gunakan kode berikut pada ESP8266:

```cpp
#define SENSOR_PIN D5
#define BUZZER_PIN D6

void setup() {
  Serial.begin(115200);
  pinMode(SENSOR_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
}

void loop() {
  int vib = digitalRead(SENSOR_PIN);
  Serial.printf("VIB:%d\n", vib);

  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    if (cmd == "BUZZ:ON") digitalWrite(BUZZER_PIN, HIGH);
    if (cmd == "BUZZ:OFF") digitalWrite(BUZZER_PIN, LOW);
  }

  delay(10);
}
```

---

# 🤖 **Kode ROS2**

## **1️⃣ publisher_vibration.py (pemegang COM)**

Fungsi:

* membaca serial COM
* publish `/vibration`
* menerima `/buzzer_cmd`
* kirim BUZZ:ON/OFF ke ESP8266

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import serial

class VibrationPublisher(Node):
    def __init__(self):
        super().__init__('vibration_publisher')

        self.ser = serial.Serial('COM10', 115200, timeout=1)
        self.pub = self.create_publisher(Bool, 'vibration', 10)
        self.sub_cmd = self.create_subscription(
            Bool, 'buzzer_cmd', self.buzzer_callback, 10
        )
        self.timer = self.create_timer(0.1, self.read_serial)

    def read_serial(self):
        if self.ser.in_waiting:
            line = self.ser.readline().decode(errors='ignore').strip()
            if line.startswith("VIB:"):
                msg = Bool()
                msg.data = (line.split(":")[1] == "1")
                self.pub.publish(msg)
                self.get_logger().info(f"[PUBLISH] Vibration: {msg.data}")

    def buzzer_callback(self, msg):
        if msg.data:
            self.ser.write(b"BUZZ:ON\n")
            self.get_logger().info("[SEND] BUZZ:ON")
        else:
            self.ser.write(b"BUZZ:OFF\n")
            self.get_logger().info("[SEND] BUZZ:OFF")

def main(args=None):
    rclpy.init(args=args)
    node = VibrationPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

---

## **2️⃣ subscriber_buzzer.py**

Fungsi:

* menerima `/vibration`
* publish `/buzzer_cmd`

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class BuzzerSubscriber(Node):
    def __init__(self):
        super().__init__('buzzer_subscriber')

        self.sub = self.create_subscription(
            Bool, 'vibration', self.cb, 10
        )
        self.pub_cmd = self.create_publisher(Bool, 'buzzer_cmd', 10)
        self.get_logger().info("Subscriber started.")

    def cb(self, msg):
        cmd = Bool()
        if msg.data:
            self.get_logger().info("[EVENT] Vibration detected → buzzer ON")
            cmd.data = True
        else:
            self.get_logger().info("[EVENT] No vibration → buzzer OFF")
            cmd.data = False
        self.pub_cmd.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = BuzzerSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

---

# ▶️ **Menjalankan Sistem**

## Terminal 1 – Publisher (serial manager)

```
cd C:\pixi_ws\ros2_ws
pixi shell
.\install\setup.ps1
ros2 run vibration_system publisher_vibration
```

## Terminal 2 – Subscriber

```
cd C:\pixi_ws\ros2_ws
pixi shell
.\install\setup.ps1
ros2 run vibration_system subscriber_buzzer
```

---

# 🧪 **Output yang Diharapkan**

### Terminal 1:

```
[PUBLISH] Vibration: True
[SEND] BUZZ:ON
```

### Terminal 2:

```
[EVENT] Vibration detected → buzzer ON
```

---

# ⚠️ Kendala Umum & Solusi

| Masalah                              | Penyebab                       | Solusi                                |
| ------------------------------------ | ------------------------------ | ------------------------------------- |
| SerialException: Access Denied       | COM sedang dipakai proses lain | Tutup semua node, kill python.exe     |
| SerialException: File Not Found      | Port COM berubah               | Cari ulang COM melalui Device Manager |
| Build gagal “Permission Denied .exe” | File .exe terkunci             | Hapus folder build/install/log        |
| Tidak ada data masuk                 | Baudrate salah                 | Pastikan baudrate 115200              |

---
