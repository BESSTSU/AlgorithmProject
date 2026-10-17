# คู่มือ M8b: ต่อหุ่นจริงสองตัวเข้ากับ ROS 2 บน WSL

เป้าหมายของขั้นนี้: เครื่อง WSL มองเห็นหุ่นทั้งสองตัว หุ่นแต่ละตัวเปิดใน namespace ของตัวเอง
และบังคับด้วยคีย์บอร์ดได้ทีละตัวโดยอีกตัวไม่ขยับ ยังไม่มีการทำแมพหรือรันแผนในขั้นนี้

> **สถานะของคู่มือนี้:** ทำผ่านแล้วกับหุ่นจริงทั้งสองตัว (2026-10-14/15): WSL โหมด mirrored เห็นหุ่นทั้งสองตัว
> เปิดด้วย namespace `tb3_0` / `tb3_1` และ teleop ได้
> **ตั้งแต่ 2026-10-15 ให้เปิดหุ่นด้วย `python -m robots.robot_up <user@IP> <namespace>`** แทน `ros2 launch` ตรงๆ
> (เปิด bringup, watchdog และ AMCL บนหุ่น ดู [m8c_guide.md](m8c_guide.md) ส่วน D) teleop ในคู่มือนี้ยังใช้ได้ตามเดิม

ชื่อที่ใช้ในคู่มือ: หุ่นตัวแรก = `tb3_0` (R1 ในไฟล์ฉาก) หุ่นตัวที่สอง = `tb3_1` (R2)
ตรงกับ `ros2.namespaces` ใน `config.yaml`

## ขั้น 0: เตรียมความปลอดภัย

- ตอนลอง teleop ครั้งแรก ให้ยกหุ่นวางบนแท่นจนล้อลอย หรือวางบนพื้นที่โล่งห่างขอบโต๊ะ
- รู้วิธีหยุดหุ่น: ในโปรแกรม teleop กด `s` หรือ `space` และปิดสวิตช์ที่ตัวหุ่นได้เสมอ
- เปิดหุ่นทีละตัวก่อน ค่อยเปิดพร้อมกัน

## ขั้น 1: ให้ WSL อยู่ในวง LAN เดียวกับหุ่น

ตอนนี้ WSL ใช้เครือข่ายแบบ NAT (ได้ IP ขึ้นต้น `172.`) หุ่นบน Wi-Fi จึงมองไม่เห็น WSL
ต้องเปลี่ยนเป็นโหมด mirrored ซึ่ง Microsoft ระบุว่ารองรับ multicast และการต่อเข้า WSL จาก LAN
(ต้องใช้ Windows 11 22H2 ขึ้นไป เครื่องนี้ผ่านเกณฑ์)

1. สร้างไฟล์ `C:\Users\User\.wslconfig` ด้วยเนื้อหานี้

   ```
   [wsl2]
   networkingMode=mirrored
   ```

2. เปิด PowerShell แล้วรีสตาร์ท WSL (ปิดทุกหน้าต่าง WSL ก่อน)

   ```powershell
   wsl --shutdown
   ```

3. เปิด Ubuntu-22.04 ใหม่ แล้วเช็ค IP

   ```bash
   hostname -I
   ```

   **ควรเห็น:** IP ของวง Wi-Fi เดียวกับ Windows (เช่น `192.168.x.x`) ไม่ใช่ `172.x.x.x`

4. เปิดให้ WSL รับการเชื่อมต่อขาเข้า ใน PowerShell แบบ Run as administrator
   (คำสั่งนี้มาจากเอกสารของ Microsoft)

   ```powershell
   Set-NetFirewallHyperVVMSetting -Name '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}' -DefaultInboundAction Allow
   ```

   **ข้อควรรู้:** คำสั่งนี้ทำให้เครื่องอื่นในวง LAN ต่อเข้ามาที่ WSL ได้ทุกพอร์ต ควรทำเฉพาะบนเครือข่ายที่ไว้ใจได้
   ยกเลิกภายหลังได้ด้วยคำสั่งเดิมโดยเปลี่ยน `Allow` เป็น `Block`

## ขั้น 2: ตั้งค่าฝั่ง WSL

เพิ่ม 2 บรรทัดนี้ท้ายไฟล์ `~/.bashrc` ใน Ubuntu-22.04 แล้วเปิดเทอร์มินัลใหม่
(ตอนนี้ไฟล์ตั้ง `ROS_DOMAIN_ID=70` ไว้แล้ว แต่ยังไม่ได้ source workspace ของ turtlebot3)

```bash
source ~/turtlebot3_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
```

เช็ค:

```bash
echo $ROS_DOMAIN_ID $TURTLEBOT3_MODEL
ros2 pkg prefix turtlebot3_teleop
```

**ควรเห็น:** `70 burger` และ path ที่อยู่ใต้ `~/turtlebot3_ws/install`

## ขั้น 3: ตรวจหุ่นทีละตัว (ทำบน Raspberry Pi ของหุ่นผ่าน SSH)

ทำกับหุ่นทั้งสองตัว แล้วจดผลลงตารางท้ายคู่มือ

```bash
echo $ROS_DOMAIN_ID $TURTLEBOT3_MODEL $LDS_MODEL
ros2 pkg xml turtlebot3_bringup -t version
ros2 launch turtlebot3_bringup robot.launch.py --show-args
```

| สิ่งที่ดู | ที่ต้องการ |
|---|---|
| `ROS_DOMAIN_ID` | `70` เท่ากับฝั่ง WSL ถ้าไม่ใช่ ให้แก้ใน `~/.bashrc` ของหุ่น |
| `TURTLEBOT3_MODEL` | `burger` |
| `LDS_MODEL` | `LDS-01` หรือ `LDS-02` ตามรุ่น LiDAR ที่ติดอยู่จริง |
| รายการ arguments ของ launch | ต้องมี `namespace` |

ถ้าไม่มี `namespace` ในรายการ แปลว่าแพ็กเกจบนหุ่นเก่ากว่ารุ่นที่รองรับ ต้องอัปเดตแพ็กเกจ turtlebot3 บนหุ่นก่อน
ให้หยุดตรงนี้แล้วแจ้งกลับ

## ขั้น 4: เปิดหุ่นตัวแรกตัวเดียว

บนหุ่นตัวแรก:

```bash
ros2 launch turtlebot3_bringup robot.launch.py namespace:=tb3_0
```

บน WSL:

```bash
ros2 topic list
```

**ควรเห็น:** topic ที่ขึ้นต้นด้วย `/tb3_0/` เช่น `/tb3_0/cmd_vel`, `/tb3_0/odom`, `/tb3_0/scan`
และ `/tf` กับ `/tf_static` ที่ไม่มีคำนำหน้า

เช็คว่าข้อมูลไหลจริง:

```bash
ros2 topic hz /tb3_0/scan
ros2 run tf2_ros tf2_echo tb3_0/odom tb3_0/base_footprint
```

**ควรเห็น:** `scan` มาสม่ำเสมอ (LDS-01 ประมาณ 5 Hz, LDS-02 ประมาณ 10 Hz)
และ `tf2_echo` พิมพ์ตำแหน่งออกมาเรื่อยๆ ชื่อ frame ต้องมีคำนำหน้า `tb3_0/`

> ชื่อ frame แบบมีคำนำหน้าเป็นสิ่งที่ผมคาดจากโค้ดของแพ็กเกจ ยังไม่เคยเห็นจากหุ่นจริง
> ถ้า frame ชื่อ `odom` กับ `base_footprint` เฉยๆ ให้จดไว้และแจ้งกลับ เพราะหุ่นสองตัวจะชนกันใน TF

ลองบังคับ (เปิดเทอร์มินัลใหม่บน WSL):

```bash
ros2 run turtlebot3_teleop teleop_keyboard --ros-args -r __ns:=/tb3_0
```

**ควรเห็น:** หุ่นขยับตามปุ่ม `w a d x` และหยุดเมื่อกด `s`

ปิดหุ่นตัวแรกด้วย Ctrl-C ที่เทอร์มินัล bringup

## ขั้น 5: เปิดหุ่นตัวที่สองตัวเดียว

ทำซ้ำขั้น 4 กับหุ่นตัวที่สอง โดยเปลี่ยน `tb3_0` เป็น `tb3_1` ทุกที่

## ขั้น 6: เปิดสองตัวพร้อมกัน

เปิด bringup บนหุ่นทั้งสองตัว (`namespace:=tb3_0` และ `namespace:=tb3_1`) แล้วบน WSL:

```bash
ros2 topic list | grep -E "cmd_vel|scan|odom"
ros2 run tf2_ros tf2_echo tb3_0/odom tb3_0/base_footprint
ros2 run tf2_ros tf2_echo tb3_1/odom tb3_1/base_footprint
```

**ควรเห็น:** topic ครบทั้งสองชุด และ `tf2_echo` ทั้งสองคำสั่งทำงานได้พร้อมกัน

จากนั้นเปิด teleop สองเทอร์มินัล (`__ns:=/tb3_0` และ `__ns:=/tb3_1`) แล้วเช็คว่า:

- บังคับ `tb3_0` แล้วมีแค่หุ่นตัวแรกที่ขยับ
- บังคับ `tb3_1` แล้วมีแค่หุ่นตัวที่สองที่ขยับ
- ตำแหน่งใน `tf2_echo` ของตัวที่ไม่ได้บังคับไม่เปลี่ยน

ถ้าผ่านทั้งสามข้อ ถือว่า 8b สำเร็จ

## ถ้า WSL มองไม่เห็น topic ของหุ่น

ไล่เช็คตามลำดับนี้:

1. `hostname -I` ใน WSL ยังขึ้น `172.` หรือไม่ ถ้าใช่ โหมด mirrored ยังไม่ทำงาน ให้เช็คไฟล์ `.wslconfig` แล้ว `wsl --shutdown` อีกครั้ง
2. `ROS_DOMAIN_ID` ของ WSL กับหุ่นตรงกันไหม
3. `ping <IP ของหุ่น>` จาก WSL ได้ไหม และ `ping <IP ของเครื่อง Windows>` จากหุ่นได้ไหม
4. ทำขั้น 1 ข้อ 4 (firewall) แล้วหรือยัง
5. เราเตอร์บางรุ่นกันไม่ให้อุปกรณ์ Wi-Fi คุยกันเอง (AP isolation) หรือไม่ส่ง multicast ลองใช้ hotspot หรือเราเตอร์อื่น

ถ้าไล่ครบแล้วยังไม่ได้ ให้แจ้งผลของแต่ละข้อ ทางสำรองคือใช้การค้นหาแบบระบุ IP ตรงๆ แทน multicast หรือรันบน Ubuntu จริง

## สิ่งที่ขอให้จดกลับมา

| รายการ | หุ่นตัวแรก (`tb3_0`) | หุ่นตัวที่สอง (`tb3_1`) |
|---|---|---|
| `LDS_MODEL` | | |
| เวอร์ชัน `turtlebot3_bringup` | | |
| launch มี argument `namespace` ไหม | | |
| ชื่อ frame ที่เห็นใน `tf2_echo` | | |
| ความถี่ของ `scan` | | |
| teleop แยกตัวได้ไหม | | |

และข้อความ error ที่เจอ ถ้ามี

## ขั้นถัดไป (8c)

เมื่อ 8b ผ่าน ขั้นต่อไปคือทำ SLAM ด้วยหุ่นตัวเดียวในสนามจริง บันทึกแมพ
แล้วเปิด `map_server` กับ AMCL สองตัวบนแมพเดียวกัน ขั้นนั้นต้องมีสนามที่สร้างเสร็จแล้ว
