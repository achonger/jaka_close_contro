#!/bin/bash

echo "==========================================="
echo "JAKA机械臂视觉伺服系统标定流程"
echo "==========================================="

echo ""
echo "步骤1: 准备标定环境"
echo "- 确保ArUco标定板固定在机械臂末端或放置在工作区域"
echo "- 确保相机视野能覆盖标定板"
echo "- 启动ROS系统和相关节点"
echo ""

read -p "请确认已准备好标定环境，按回车继续..."

echo ""
echo "步骤2: 启动标定相关节点"
echo "roslaunch jaka_close_contro hand_eye_calibration.launch"
echo ""
echo "注意: 在新终端中运行上述命令启动系统"
read -p "系统启动完成后按回车继续..."

echo ""
echo "步骤3: 收集标定数据"
echo "在不同位姿下收集数据点，建议至少收集10个不同位姿的数据"
echo ""

for i in {1..10}
do
    echo "数据点 $i/10"
    read -p "将机械臂移动到新位姿并确保能检测到标记，完成后输入 'c' 收集数据: " input
    if [ "$input" = "c" ]; then
        rosservice call /collect_calibration_data "{}"
        echo "数据点 $i 已收集"
    else
        echo "跳过数据点 $i"
    fi
done

echo ""
echo "步骤4: 执行手眼标定"
read -p "所有数据收集完成后，按回车执行标定: "
rosservice call /calibrate_hand_eye "{}"

echo ""
echo "步骤5: 验证标定结果"
echo "移动机械臂到不同位置，观察检测到的物体位姿是否与实际相符"
read -p "验证完成后按回车继续..."

echo ""
echo "步骤6: 设置控制目标位姿"
echo "编辑closed_loop_control_node.cpp中的target_pose_变量设置目标位姿"
echo "或者通过参数服务器动态设置"
read -p "设置目标位姿后按回车开始闭环控制: "

echo ""
echo "步骤7: 执行闭环控制"
echo "rosservice call /start_closed_loop_control \"{}\""
echo ""
echo "闭环控制将自动执行，直到物体到达目标位姿或达到最大迭代次数"

echo ""
echo "==========================================="
echo "标定流程完成！"
echo "==========================================="