## โปรเจคนี้เป็นส่วนหนึ่งของวิชา Autonomous Robot System (271411)
### จัดทำโดย นายภูมิพัฒน์ วัฒนประสิทธิ์
### รหัสนักศึกษา 640610682
### คณะวิศวกรรมศาสตร์ มหาวิทยาลัยเชียงใหม่

## ขั้นตอนการติดตั้ง workspace
### 1. git clone https://github.com/PoommipatWat/autonomous_robotics_system_640610682.git
### 2. cd autonomous_robotics_system_640610682
### 3. colcon build
### 4. source install/setup.bash

## ขั้นตอนการสร้างแผนที่
### ros2 launch ai_robot keep_map.launch.py

## ขั้นตอนการโหลดแผนที่ ที่เก็บมา และใช้งาน rrt*
### ros2 launch ai_robot bringup.launch.py

## ขั้นตอนการโหลดแผนที่ ที่ถูกเก็บมาเป็นตัวอย่าง และใช้งาน rrt*
### ros2 launch ai_robot example.launch.py