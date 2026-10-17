# คู่มือ M8c: แมพสำหรับระบุตำแหน่ง และระบุตำแหน่งหุ่นสองตัวบนแมพเดียวกัน

เป้าหมายของขั้นนี้: มีแมพของสนามจริงหนึ่งไฟล์ที่ตรงกับกริดของ planner
และหุ่นทั้งสองตัวระบุตำแหน่งตัวเองบนแมพเดียวกันได้

> **วิธีที่ใช้ (เปลี่ยนเมื่อ 2026-10-14):** ไม่ได้เก็บแมพด้วย SLAM แต่**วาดแมพจากไฟล์กริด** `maps/arena_6x5.txt`
> เพราะลอง cartographer บนสนามจริงแล้ว แมพเพี้ยนในทางเดินแถว 2 ทั้งที่ odom ของหุ่นแม่นดี (รายละเอียดใน SPEC หัวข้อ 10.4)
> แมพที่วาดตรงกับสนามจริงมาก จุด scan ของหุ่นตัวแรกห่างกำแพงในแมพค่ากลาง 0.8 cm และ 90% ไม่เกิน 2 cm
> (รูป [layout_map_scan_check.png](layout_map_scan_check.png))
>
> **ทดสอบแล้วกับหุ่นจริงทั้งสองตัว (2026-10-16)** ด้วยวิธีในส่วน D: `robot_up` เปิด AMCL บนหุ่นแต่ละตัว
> ส่วน C (เปิด AMCL บนโน้ตบุ๊ก) เป็นวิธีเดิม ใช้ได้กับหุ่นตัวเดียวบนเครือข่ายที่ดี แต่บน hotspot ที่ช้าระบุตำแหน่งไม่นิ่ง

ก่อนเริ่ม โน้ตบุ๊กต้องเห็นหุ่นผ่าน ROS แล้ว ทำตาม [setup_before_mapping.md](setup_before_mapping.md) ขั้น 1–6
และการเปิดหุ่นด้วย namespace ดู [m8b_guide.md](m8b_guide.md)

## ส่วน A: สร้างไฟล์แมพ (ทำใหม่เมื่อแก้ไฟล์กริดหรือขนาดช่อง)

ในโฟลเดอร์โปรเจค (Windows หรือ WSL ก็ได้)

```bash
python -m robots.layout_map
```

**ควรเห็น:** `wrote maps/ros/arena_6x5.pgm (310x265 px) and maps/ros/arena_6x5.yaml`

ค่าที่ใช้มาจาก `config.yaml`

| ค่า | ความหมาย |
|---|---|
| `cell_size: 0.45` | ด้านของช่องหนึ่งช่อง วัดจากสนามจริง |
| `ros2.layout_map.wall_thickness: 0.01` | ความหนากำแพง 1 cm |
| `ros2.layout_map.resolution: 0.01` | แมพละเอียด 1 cm ต่อพิกเซล |
| `ros2.grid_origin: {x: 0, y: 0, yaw: 0}` | มุมซ้ายล่างของกริดอยู่ที่จุด (0,0) ของแมพพอดี ไม่ต้อง calibrate |

พิกัดกลางช่องในแมพ: x = 0.45 × (คอลัมน์ + 0.5), y = 0.45 × (4.5 − แถว) เช่น (2,0) อยู่ที่ (0.225, 1.125)

## ส่วน B: build แพ็กเกจ (ทำครั้งเดียว ทำแล้วเมื่อ 2026-10-14)

ใน **เทอร์มินัล WSL**

```bash
mkdir -p ~/mrpp_ws/src
ln -s /mnt/g/Year4/271401-Artificial_Intelligence_for_Robotics/project/ros2/mrpp_bringup ~/mrpp_ws/src/
cd ~/mrpp_ws && colcon build
```

**ควรเห็น:** `Summary: 1 package finished`

ถ้าไม่อยากพิมพ์ `source` ทุกครั้ง ให้เพิ่มบรรทัดนี้ท้าย `~/.bashrc` แล้วเปิดเทอร์มินัลใหม่

```bash
source ~/mrpp_ws/install/setup.bash
```

## ส่วน C: ตรวจหุ่นหนึ่งตัวบนแมพ (วิธีเดิม: AMCL บนโน้ตบุ๊ก)

ถ้าใช้ส่วน D (`robot_up`) ไม่ต้องทำส่วนนี้ และห้ามเปิด localization บนโน้ตบุ๊กพร้อมกับบนหุ่น

1. **เทอร์มินัลหุ่น** เปิดหุ่นด้วย namespace

   ```bash
   ros2 launch turtlebot3_bringup robot.launch.py namespace:=tb3_0
   ```

   **ควรเห็น:** `[tb3_0.turtlebot3_node]: Run!`

2. **เทอร์มินัล WSL** เปิดแมพและ AMCL ของหุ่นตัวนี้ตัวเดียว

   ```bash
   source ~/mrpp_ws/install/setup.bash
   ros2 launch mrpp_bringup localization.launch.py map:=/mnt/g/Year4/271401-Artificial_Intelligence_for_Robotics/project/maps/ros/arena_6x5.yaml namespaces:=tb3_0
   ```

   ห้ามเปิด cartographer ค้างไว้พร้อมกัน เพราะมันส่ง `/map` และ TF `map` ชนกัน

3. วางหุ่นกลางช่อง (2,0) หันไปทางช่องชาร์จ แล้วส่งตำแหน่งเริ่มต้นจาก RViz ด้วยปุ่ม 2D Pose Estimate
   (ใน RViz ตั้ง Tool Properties ของปุ่มนี้ให้ topic เป็น `/tb3_0/initialpose`)
   หรือใช้ `--localize` ในส่วน D ก็ได้

4. เช็คด้วยตาใน RViz (`rviz2`): Fixed Frame เป็น `map` เพิ่ม Map (`/map`) และ LaserScan (`/tb3_0/scan`)

   **ควรเห็น:** จุดเลเซอร์ทับเส้นกำแพงของแมพ

## ส่วน D: ระบุตำแหน่งหุ่นสองตัว

1. เปิดหุ่นทั้งสองตัวด้วย namespace **พร้อม watchdog และ AMCL บนหุ่น** จากโน้ตบุ๊ก (PowerShell ในโฟลเดอร์โปรเจค)
   ครั้งแรกต้องติดตั้ง `ros-humble-nav2-amcl ros-humble-nav2-map-server ros-humble-nav2-lifecycle-manager` บนหุ่นก่อน (ใช้ sudo)

   ```bash
   python -m robots.robot_up turtlebest1@<IP ตัวแรก> tb3_0
   ```

   ```bash
   python -m robots.robot_up turtlebest2@<IP ตัวที่สอง> tb3_1
   ```

   **ควรเห็น:** `bringup, watchdog and localization running` ทั้งสองตัว

   ตั้งแต่ 2026-10-15 โปรแกรมบนโน้ตบุ๊กส่งคำสั่งไปที่ `cmd_vel_remote` ซึ่งมีแค่ watchdog รับ
   ถ้าเปิด bringup เองด้วย `ros2 launch` แบบไม่มี watchdog หุ่นจะไม่ขยับตามแผน (แต่ teleop ยังใช้ได้)
   ถ้า IP ของหุ่นเปลี่ยน ให้ ssh เข้าไปหนึ่งครั้งก่อน เพื่อยอมรับ host key ของ IP ใหม่

2. **วางหุ่นกลางช่องเริ่มต้นของฉาก S0 และหันหน้าไปทางช่องชาร์จ**

   | หุ่น | namespace | ช่องเริ่มต้น |
   |---|---|---|
   | R1 | `tb3_0` | (2,0) ปลายซ้ายของทางเดิน |
   | R2 | `tb3_1` | (2,4) ช่องติดกับช่องชาร์จ |

3. (ไม่ต้องเปิด AMCL บนโน้ตบุ๊ก `robot_up` เปิดบนหุ่นให้แล้ว)

4. ตรวจก่อนออกตัว คำสั่งนี้ส่งตำแหน่งเริ่มต้นให้ AMCL แล้วรายงานผล **ไม่สั่งให้หุ่นขยับ**
   รันในโฟลเดอร์โปรเจคบน WSL

   ```bash
   python3 -m robots.turtlebot_ros2 --scenario scenarios/s0.yaml --localize
   ```

   **ควรเห็น:**

   ```
   R1: localized on (2, 0), ... cm from the center of start cell (2, 0) -> OK
   R2: localized on (2, 4), ... cm from the center of start cell (2, 4) -> OK
   ```

5. เช็คด้วยตาใน RViz: Fixed Frame `map` เพิ่ม Map (`/tb3_0/map`) และ LaserScan `/tb3_0/scan`, `/tb3_1/scan`
   **สำคัญ:** ถ้าหุ่นวางหันกลับด้าน `--localize` ยังขึ้น OK ได้ ต้องดูว่าจุดเลเซอร์ทับกำแพงจริง

ถ้าผ่านข้อ 4 และ 5 ถือว่า 8c สำเร็จ

## ซ้อมด้วยหุ่นจริงตัวเดียวกับหุ่นจำลองตัวที่ 2

ใช้ตอนที่ยังมีหุ่นจริงพร้อมแค่ตัวเดียว ตัวอย่างนี้ใช้ R1 (`tb3_0`) เป็นหุ่นจริง ส่วน R2 (`tb3_1`) เป็นหุ่นในจินตนาการ
ที่ `fake_turtlebot` จำลองให้ ซึ่งรับคำสั่งแบบเดียวกับหุ่นจริง และส่งตำแหน่งผ่าน TF
planner เห็นหุ่นสองตัวตามปกติ แต่ LiDAR ของหุ่นจริงจะมองไม่เห็นหุ่นจำลอง
สลับกันได้: ใช้ R2 จริงกับ `--only tb3_0` (ทดสอบแล้วทั้งสองแบบ 2026-10-15)

1. เปิดหุ่นจริงด้วย `robot_up` ตามส่วน D ข้อ 1 (เฉพาะตัวที่ใช้) ซอฟต์แวร์บนหุ่นอีกตัวต้องปิดอยู่ (`--down`)
2. วางหุ่นจริงกลางช่องเริ่มต้นของมัน หันไปทางช่องชาร์จ และ**ยกหุ่นตัวที่ไม่ได้ใช้ออกจากสนาม**
   (หุ่นจำลองจะวิ่งผ่านช่องที่หุ่นตัวนั้นจอดอยู่)
3. **เทอร์มินัล WSL** เปิดหุ่นจำลองตัวที่ 2 ในโฟลเดอร์โปรเจค

   ```bash
   python3 -m robots.fake_turtlebot --scenario scenarios/s0.yaml --only tb3_1
   ```

4. ตรวจก่อนออกตัวด้วยคำสั่งในส่วน D ข้อ 4 ทั้ง R1 และ R2 ควรขึ้น `OK`
5. รันฉาก **หุ่นจริงจะเคลื่อนที่** ให้มีคนอยู่ใกล้สวิตช์หุ่น

   ```bash
   python3 -m robots.turtlebot_ros2 --scenario scenarios/s0.yaml
   ```

## ถ้าผลไม่ตรง

| อาการ | สาเหตุที่น่าจะเป็น |
|---|---|
| `nothing listens on /tb3_0/initialpose` | ยังไม่ได้เปิด launch ในส่วน C/D หรือ `ROS_DOMAIN_ID` ไม่ตรงกัน |
| `nothing listens on /tb3_1/initialpose` ตอนซ้อมกับหุ่นจำลอง | ยังไม่ได้เปิด `fake_turtlebot --only tb3_1` |
| `no pose in frame 'map' for [...]` | AMCL ยังไม่ได้ `scan` หรือ TF ของหุ่นตัวนั้น เช็ค `ros2 topic hz /tb3_0/scan` และชื่อ frame ตามขั้น 8b |
| `NOT on its start cell` และระยะคลาดมาก | หุ่นไม่ได้วางบนช่องเริ่มต้น หรือสนามจริงเหลื่อมจากแบบ |
| จุดเลเซอร์หมุนเอียงจากกำแพง | หุ่นไม่ได้หันหน้าไปทางช่องชาร์จตอนส่งตำแหน่งเริ่มต้น |
| จุดเลเซอร์เลื่อนจากกำแพงเท่าๆ กันทั้งแมพ | ตำแหน่งวางหุ่นคลาด ลองขยับหุ่นเล็กน้อยให้ AMCL ปรับ หรือวางใหม่แล้วรันข้อ 4 อีกครั้ง |
| จุดเลเซอร์ไม่ทับกำแพงบางเส้นตลอด ไม่ว่าวางหุ่นที่ไหน | กำแพงจริงเหลื่อมจากแบบ แก้สนาม หรือวัดจุดอ้างอิงแล้วใช้ `python3 -m robots.calibrate_grid` หา `grid_origin` ใหม่ |

ถ้าหุ่นวางหันทิศอื่น แก้ค่า `ros2.initial_pose.heading` ใน `config.yaml` (หน่วยเรเดียน วัดจากทิศที่คอลัมน์เพิ่มขึ้น)

## ขั้นถัดไป (8d)

เมื่อ 8c ผ่าน คำสั่งรันฉาก S0 บนหุ่นจริงคือ

```bash
python3 -m robots.turtlebot_ros2 --scenario scenarios/s0.yaml
```

โปรแกรมจะไม่ออกตัวถ้าหุ่นไม่ได้อยู่ในช่องเริ่มต้น และกด Ctrl-C เมื่อไรก็สั่งหยุดหุ่นทั้งสองตัว
รอบแรกควรมีคนยืนใกล้สวิตช์ของหุ่นแต่ละตัว
