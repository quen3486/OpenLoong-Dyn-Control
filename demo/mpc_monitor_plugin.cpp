/*
 * MPC Monitor Plugin - 用于实时监控 QP 求解状态和物理效果
 * 使用方式：将此代码片段添加到 walk_mpc_wbc_v4.cpp 的主循环中
 */

// ===== 在 DataLogger 初始化后添加 =====
logger.addIterm("mpc_qpStatus", 1);
logger.addIterm("mpc_qpCpuTime_ms", 1);
logger.addIterm("mpc_qpNWSR", 1);
logger.addIterm("com_pos_x", 1);
logger.addIterm("com_pos_y", 1);
logger.addIterm("com_pos_z", 1);
logger.addIterm("com_vel_x", 1);
logger.addIterm("com_vel_y", 1);
logger.addIterm("com_vel_z", 1);
logger.addIterm("fr_l_z", 1);
logger.addIterm("fr_r_z", 1);

// ===== 在主循环中（WBC 计算后）添加日志记录 =====
logger.recItermData("mpc_qpStatus", RobotState.qpStatus_MPC);
logger.recItermData("mpc_qpCpuTime_ms", RobotState.qp_cpuTime_MPC * 1000.0);  // 转换为 ms
logger.recItermData("mpc_qpNWSR", RobotState.qp_nWSR_MPC);
logger.recItermData("com_pos_x", RobotState.pCoM_W(0));
logger.recItermData("com_pos_y", RobotState.pCoM_W(1));
logger.recItermData("com_pos_z", RobotState.pCoM_W(2));
logger.recItermData("com_vel_x", RobotState.base_vel(0));
logger.recItermData("com_vel_y", RobotState.base_vel(1));
logger.recItermData("com_vel_z", RobotState.base_vel(2));
logger.recItermData("fr_l_z", RobotState.Fr_ff(2));   // 左脚期望接触力
logger.recItermData("fr_r_z", RobotState.Fr_ff(8));   // 右脚期望接触力

// ===== 实时打印 QP 状态（每 100 步打印一次）=====
static int printCounter = 0;
printCounter++;
if (printCounter % 100 == 0) {
    std::cout << "[MPC Monitor] Status=" << RobotState.qpStatus_MPC 
              << " CPU=" << RobotState.qp_cpuTime_MPC * 1000.0 << "ms"
              << " NWSR=" << RobotState.qp_nWSR_MPC
              << " CoM_Z=" << RobotState.pCoM_W(2)
              << std::endl;
    
    // QP 失败警告
    if (RobotState.qpStatus_MPC != 0) {
        std::cerr << "[WARNING] QP solve failed! Status=" << RobotState.qpStatus_MPC << std::endl;
    }
    
    // CPU 时间警告
    if (RobotState.qp_cpuTime_MPC > 0.004) {  // > 4ms
        std::cerr << "[WARNING] QP CPU time too high: " 
                  << RobotState.qp_cpuTime_MPC * 1000.0 << "ms" << std::endl;
    }
}
