using System;
using System.Collections.Generic;
using System.Text;
using System.Runtime.InteropServices;

namespace NimServoSDK_DLL
{
    public enum CanBaudRate
    {
        CAN_BT_10K = 0,
        CAN_BT_20K,
        CAN_BT_50K,
        CAN_BT_100K,
        CAN_BT_125K,
        CAN_BT_250K,
        CAN_BT_500K,
        CAN_BT_800K,
        CAN_BT_1000K
    };

    public enum ServoWorkMode
    {
        SERVO_PP_MODE = 1,  //轮廓位置模式
        SERVO_VM_MODE = 2,  //速度模式
        SERVO_PV_MODE = 3,  //轮廓速度模式
        SERVO_PT_MODE = 4,  //轮廓转矩模式
        SERVO_HM_MODE = 6,  //原点回归模式
        SERVO_IP_MODE = 7,  //位置插补模式
        SERVO_CSP_MODE,     //循环同步位置模式
        SERVO_CSV_MODE,     //循环同步速度模式
        SERVO_CST_MODE      //循环同步转矩模式
    };

    
	public enum ServoSDK_Error
	{
		ServoSDK_NoError = 0,           // 0没有错误
		ServoSDK_NotRegisted,           // 1SDK没有注册			（当前无需注册）
		ServoSDK_NotInitialized,        // 2SDK没有初始化		（未执行初始化函数）
		ServoSDK_UnsupportedCommType,   // 3不支持的通信方式	（缺少bin路径下相关文件）
		ServoSDK_ParamError,            // 4输入参数错误		（通信连接字符串填写错误）
		ServoSDK_CreateMasterFailed,    // 5创建主站失败		（缺少对应主站）
		ServoSDK_MasterNotExist,        // 6主站不存在			
		ServoSDK_MasterStartFailed,     // 7主站启动失败
		ServoSDK_MasterNotRunning,      // 8主站未运行
		ServoSDK_SlaveNotOnline,        // 9从站不在线
		ServoSDK_LoadParamSheetFailed,  //10加载参数表错误		（缺少数据库文件）
		ServoSDK_ParamNotExist,         //11请求的参数不存在	（对象字典填写错误）
		ServoSDK_ReadSDOFailed,         //12读SDO失败
		ServoSDK_WriteSDOFailed,        //13写SDO失败
		ServoSDK_OperationNotAllowed,   //14操作不允许			（电机使能不能进行相关操作）
		ServoSDK_MasterInternalError,   //15主站内部错误		（无法切换操作、预操作模式）
		ServoSDK_SlaveInternalError,    //16从站内部错误		（H6041处于故障状态）
		ServoSDK_Cia402ModeError,       //17从站402模式错误		（H6060与实际运动不符）
		ServoSDK_ReadWorkModeFailed,    //18读取工作模式失败	（对象字典H6060）
		ServoSDK_ReadStatusWordFailed,  //19读取状态字失败		（对象字典H6041）
		ServoSDK_ReadCurrentPosFailed,  //20读取当前位置失败	（对象字典H6064）
		ServoSDK_ReadRPDOConfigFailed,  //21读取PDO配置失败	
		ServoSDK_ReadTPDOConfigFailed,  //22读取PDO配置失败
		ServoSDK_WriteControlWordFailed,//23写控制字失败		（对象字典H6040）
		ServoSDK_WriteTargetPosFailed,  //24写目标位置失败		（对象字典H607A）
		ServoSDK_WriteTargetVelFailed,  //25写目标速度失败		（对象字典H60FF）
		ServoSDK_WriteGoHomeTypeFailed, //26写原点回归方式失败	（对象字典H6098）
		ServoSDK_GetHostInfoFailed,     //27获取主机信息失败	（无需注册不再使用）
		ServoSDK_SaveParamsFailed,      //28保存参数失败
		ServoSDK_NoAvailableDevice,     //29没有可用的设备
		ServoSDK_Unknown = 255          //255未知错误
	};

    public static class NimServoSDK
    {
        /**
         * @brief SDK初始化
         * @param nCommType 通信方式：0 CANopen；1 EtherCAT ； 2 Modbus
         * @return 0 成功；其它 失败
         */
        [DllImport("NimServoSDK.dll", EntryPoint = "Nim_init")]
        public static extern int Nim_init(string strSdkPath);

        /**
         * @brief SDK 反初始化
         */
        [DllImport("NimServoSDK.dll", EntryPoint = "Nim_clean")]
        public static extern void Nim_clean();

        /**
         * @brief 设置日志输出标志
         * @param bit0=1输出到控制台；bit1=1输出到文件
         */
        [DllImport("NimServoSDK.dll", EntryPoint = "Nim_setLogFlags")]
        public static extern void Nim_setLogFlags(int nFlags);

        /**
         * @brief 获取日志输出标志
         * @return bit0=1输出到控制台；bit1=1输出到文件
         */
        [DllImport("NimServoSDK.dll", EntryPoint = "Nim_getLogFlags")]
        public static extern int Nim_getLogFlags();

        /**
         * @brief 创建主站对象
         * @param nCommType 通信方式：0 CANopen；1 EtherCAT; 2 Modbus
         * @param handle 输出参数，成功时返回创建的主站对象句柄
         * @return 0 成功；其它 失败
         */
        [DllImport("NimServoSDK.dll", EntryPoint = "Nim_create_master")]
        public static extern int Nim_create_master(int nCommType, ref uint handle);

        /**
         * @brief 销毁主站对象
         * @param handle 由Nim_create_master函数创建的主站对象句柄
         * @return 0 成功；其它 失败
         */
        [DllImport("NimServoSDK.dll", EntryPoint = "Nim_destroy_master")]
        public static extern int Nim_destroy_master(uint handle);

        /**
         * @brief 启动通信主站
         * @param conn_str 连接字符串，json格式，具体内容参见使用手册
         * @return 0 成功；其它 失败
         */
        [DllImport("NimServoSDK.dll", EntryPoint = "Nim_master_run")]
        public static extern int Nim_master_run(uint hMaster, string conn_str);

        /**
         * @brief 关闭通信主站
         * @return 0 成功；其它 失败
         */
        [DllImport("NimServoSDK.dll", EntryPoint = "Nim_master_stop")]
        public static extern int Nim_master_stop(uint hMaster);

        /**
         * @brief 主站进入OP模式
         * @return 0 成功；其它 失败
         */
        [DllImport("NimServoSDK.dll", EntryPoint = "Nim_master_changeToOP")]
        public static extern int Nim_master_changeToOP(uint hMaster);

        /**
         * @brief 主站进入PreOP模式
         * @return 0 成功；其它 失败
         */
        [DllImport("NimServoSDK.dll", EntryPoint = "Nim_master_changeToPreOP")]
        public static extern int Nim_master_changeToPreOP(uint hMaster);

        /**
         * @brief 按照指定的地址范围扫描从站是否在线
         * @param from 起始地址
         * @param to 结束地址
         * @return 0 成功；其它 失败
         */
        [DllImport("NimServoSDK.dll", EntryPoint = "Nim_scan_nodes")]
        public static extern int Nim_scan_nodes(uint hMaster, int from, int to);

        /**
         * @brief 查询从站是否在线
         * @param nodeId 从站地址
         * @return 非零 在线；0 不在线
         */
        [DllImport("NimServoSDK.dll", EntryPoint = "Nim_is_online")]
        public static extern int Nim_is_online(uint hMaster, int nodeId);

        /**
         * @brief 读取从站PDO配置
         * @return 0 成功；其它 失败
         */
        [DllImport("NimServoSDK.dll", EntryPoint = "Nim_read_PDOConfig")]
        public static extern int Nim_read_PDOConfig(uint hMaster, int nodeId);

        /**
         * @brief 加载电机参数表
         * @param nodeId 从站地址
         * @param db_name 参数表数据库文件名
         * @return 0 成功；其它 失败
         */
        [DllImport("NimServoSDK.dll", EntryPoint = "Nim_load_params")]
        public static extern int Nim_load_params(uint hMaster, int nodeId, string db_name);

        /**
         * @brief 读取从站参数
         * @param nodeId 从站地址
         * @param pParamNO 参数编号
         * @param puiValue 参数值
         * @param bSDO 1 使用SDO读；0 使用PDO
         * @return 0 成功；其它 失败
         */
        [DllImport("NimServoSDK.dll", EntryPoint = "Nim_get_param_value")]
        public static extern int Nim_get_param_value(uint hMaster, int nodeId, string ParamNO, ref uint puiValue, int bSDO);

        /**
         * @brief 设置从站参数
         * @param nodeId 从站地址
         * @param pParamNO 参数编号
         * @param uiValue 参数值
         * @param bSDO 1 使用SDO写；0 使用PDO
         * @return 0 成功；其它 失败
         */
        [DllImport("NimServoSDK.dll", EntryPoint = "Nim_set_param_value")]
        public static extern int Nim_set_param_value(uint hMaster, int nodeId, string ParamNO, uint uiValue, int bSDO);

        /**
         * @brief 电机抱机
         * @param nodeId 从站地址
         * @param bSDO 1 使用SDO控制；0 使用PDO
         * @return 0 成功；其它 失败
         */
        [DllImport("NimServoSDK.dll", EntryPoint = "Nim_power_on")]
        public static extern int Nim_power_on(uint hMaster, int nodeId, int bSDO);

        /**
         * @brief 电机脱机
         * @param nodeId 从站地址
         * @param bSDO 1 使用SDO控制；0 使用PDO
         * @return true 成功；false 失败
         */
        [DllImport("NimServoSDK.dll", EntryPoint = "Nim_power_off")]
        public static extern int Nim_power_off(uint hMaster, int nodeId, int bSDO);

        /**
         * @brief 设置控制字(6040)
         * @param nodeId 从站地址
         * @param cw 控制字
         * @param bSDO 1 使用SDO写；0 使用PDO
         * @return 0 成功；其它 失败
         */
        [DllImport("NimServoSDK.dll", EntryPoint = "Nim_set_controlWord")]
        public static extern int Nim_set_controlWord(uint hMaster, int nodeId, UInt16 cw, int bSDO);

        /**
         * @brief 获取电机状态字(6041)
         * @param nodeId 电机地址
         * @param sw 输出参数，成功时返回电机状态字
         * @param bSDO 1 使用SDO写；0 使用PDO
         * @return 0 成功；其它 失败
         */
        [DllImport("NimServoSDK.dll", EntryPoint = "Nim_get_statusWord")]
        public static extern int Nim_get_statusWord(uint hMaster, int nodeId, ref UInt16 sw, int bSDO);

        /**
         * @brief 设置电机工作模式（6060,在脱机状态下设置）
         * @param nodeId 电机地址
         * @param mode 模式
         * @param bSDO 1 使用SDO写；0 使用PDO
         * @return 0 成功；其它 失败
         */
        [DllImport("NimServoSDK.dll", EntryPoint = "Nim_set_workMode")]
        public static extern int Nim_set_workMode(uint hMaster, int nodeId, int mode, int bSDO);

        /**
         * @brief 获取电机工作模式显示值(6061)
         * @param nodeId 电机地址
         * @param mode 输出参数，成功时返回电机工作模式
         * @param bSDO 1 使用SDO写；0 使用PDO
         * @return 0 成功；其它 失败
         */
        [DllImport("NimServoSDK.dll", EntryPoint = "Nim_get_workModeDisplay")]
        public static extern int Nim_get_workModeDisplay(uint hMaster, int nodeId, ref int mode, int bSDO);

		/**
		 * @brief 设置原点回归方式
		 * @param hMaster 主站对象句柄
		 * @param nodeId 轴地址
		 * @param type 回原点方式
		 * @return 0 成功；其它 失败
		 */
		[DllImport("NimServoSDK.dll")]
		public static extern int Nim_set_homeType(uint hMaster, int nodeId, byte type);

        /**
		 * @brief 获取原点回归方式
		 * @param hMaster 主站对象句柄
		 * @param nodeId 轴地址
		 * @param type 回原点方式
		 * @return 0 成功；其它 失败
		 */
        [DllImport("NimServoSDK.dll")]
        public static extern int Nim_get_homeType(uint hMaster, int nodeId, byte[] type);

        /**
         * @brief 原点回归
         * @param nodeId 轴地址
         * @param bSDO 1 使用SDO控制；0 使用PDO
         * @return 0 成功；其它 失败
         */
        [DllImport("NimServoSDK.dll", EntryPoint = "Nim_goHome")]
        public static extern int Nim_goHome(uint hMaster, int nodeId, int bSDO);

        /**
         * @brief 轮廓速度模式下正转
         * @param nodeId 轴地址
         * @param fVelocity  速度（用户单位/s）
         * @param bSDO 1 使用SDO控制；0 使用PDO
         * @return 0 成功；其它 失败
         */
        [DllImport("NimServoSDK.dll", EntryPoint = "Nim_forward")]
        public static extern int Nim_forward(uint hMaster, int nodeId, double fVelocity, int bSDO);

        /**
         * @brief 轮廓速度模式下反转
         * @param nodeId 轴地址
         * @param fVelocity  速度（用户单位/s）
         * @param bSDO 1 使用SDO控制；0 使用PDO
         * @return 0 成功；其它 失败
         */
        [DllImport("NimServoSDK.dll", EntryPoint = "Nim_backward")]
        public static extern int Nim_backward(uint hMaster, int nodeId, double fVelocity, int bSDO);

        /**
         * @brief 设置目标速度(60FF)
         * @param nodeId 从站地址
         * @param fVelocity 目标速度（用户单位/s）
         * @param bSDO 1 使用SDO写；0 使用PDO
         * @return 0 成功；其它 失败
         */
        [DllImport("NimServoSDK.dll", EntryPoint = "Nim_set_targetVelocity")]
        public static extern int Nim_set_targetVelocity(uint hMaster, int nodeId, double fVelocity, int bSDO);

        /**
         * @brief 设置VM模式下的目标速度
         * @param nodeId 从站地址
         * @param speed 目标速度（rpm）
         * @param bSDO 1 使用SDO写；0 使用PDO
         * @return 0 成功；其它 失败
         */
        [DllImport("NimServoSDK.dll", EntryPoint = "Nim_set_vmTargetSpeed")]
        public static extern int Nim_set_vmTargetSpeed(uint hMaster, int nodeId, int speed, int bSDO);

        /**
         * @brief 获取VM模式下的当前速度
         * @param nodeId 从站地址
         * @param speed 输出参数，成功时返回当前速度（rpm）
         * @param bSDO 1 使用SDO读；0 使用PDO
         * @return 0 成功；其它 失败
         */
        [DllImport("NimServoSDK.dll", EntryPoint = "Nim_get_vmCurrentSpeed")]
        public static extern int Nim_get_vmCurrentSpeed(uint hMaster, int nodeId, ref int speed, int bSDO);

        /**
         * @brief 轮廓位置模式下绝对位置运动
         * @param nodeId 轴地址
         * @param target 目标位置（用户单位）
         * @param bChangeImmediatly 是否立即更新：1 立即更新；0 非立即更新
         * @param bSDO 1 使用SDO写；0 使用PDO
         * @return 0 成功；其它 失败
         */
        [DllImport("NimServoSDK.dll", EntryPoint = "Nim_moveAbsolute")]
        public static extern int Nim_moveAbsolute(uint hMaster, int nodeId, double position, int bChangeImmediatly, int bSDO);

        /**
         * @brief 轮廓位置模式下相对位置运动
         * @param nodeId 轴地址
         * @param distance 目标位置（用户单位）
         * @param bChangeImmediatly 是否立即更新：1 立即更新；0 非立即更新
         * @param bSDO 1 使用SDO写；0 使用PDO
         * @return 0 成功；其它 失败
         */
        [DllImport("NimServoSDK.dll", EntryPoint = "Nim_moveRelative")]
        public static extern int Nim_moveRelative(uint hMaster, int nodeId, double distance, int bChangeImmediatly, int bSDO);

        /**
         * @brief 设置目标位置(607A)
         * @param nodeId 从站地址
         * @param fPos 目标位置（用户单位）
         * @param bSDO 1 使用SDO写；0 使用PDO
         * @return 0 成功；其它 失败
         */
        [DllImport("NimServoSDK.dll", EntryPoint = "Nim_set_targetPosition")]
        public static extern int Nim_set_targetPosition(uint hMaster, int nodeId, double fPos, int bSDO);

        /**
         * @brief 设置插补位置(60C1:01)
         * @param nodeId 从站地址
         * @param fPos  位置（用户单位）
         * @param bSDO 1 使用SDO写；0 使用PDO
         * @return 0 成功；其它 失败
         */
        [DllImport("NimServoSDK.dll", EntryPoint = "Nim_set_ipPosition")]
        public static extern int Nim_set_ipPosition(uint hMaster, int nodeId, double fPos, int bSDO);
		
		/**
		 * @brief 设置插补位置(60C2:01, 60C2:02)
		 * @param hMaster 主站对象句柄
		 * @param nodeId 从站地址
		 * @param nPeriodMS  插补周期（单位：ms）
		 * @return 0 成功；其它 失败
		 */
        [DllImport("NimServoSDK.dll", EntryPoint = "Nim_set_ipPeriod")]
		public static extern int Nim_set_ipPeriod(uint hMaster, int nodeId, uint nPeriodMS);

		/**
		 * @brief 获取插补位置(60C2:01, 60C2:02)
		 * @param hMaster 主站对象句柄
		 * @param nodeId 从站地址
		 * @param nPeriodMS  插补周期（单位：ms）
		 * @return 0 成功；其它 失败
		 */
        [DllImport("NimServoSDK.dll", EntryPoint = "Nim_get_ipPeriod")]
		public static extern int Nim_get_ipPeriod(uint hMaster, int nodeId, ref uint nPeriodMS);

        /**
         * @brief 设置目标转矩(6071)
         * @param nodeId 从站地址
         * @param torque  目标转矩（0.001倍额定转矩）
         * @param bSDO 1 使用SDO写；0 使用PDO
         * @return 0 成功；其它 失败
         */
        [DllImport("NimServoSDK.dll", EntryPoint = "Nim_set_targetTorque")]
        public static extern int Nim_set_targetTorque(uint hMaster, int nodeId, int torque, int bSDO);

        /**
         * @brief 获取当前转矩(6077)
         * @param nodeId 从站地址
         * @param torque 输出参数，成功时返回当前转矩（0.001倍额定转矩）
         * @param bSDO 1 使用SDO写；0 使用PDO
         * @return 0 成功；其它 失败
         */
        [DllImport("NimServoSDK.dll", EntryPoint = "Nim_get_currentTorque")]
        public static extern int Nim_get_currentTorque(uint hMaster, int nodeId, ref int torque, int bSDO);

		/**
		 * @brief 设置轮廓转矩模式下速度限制(2007:10h、2207:11h)
		 * @param hMaster 主站对象句柄
		 * @param nodeId 从站地址
		 * @param FwrSpeedLimit  正向速度限制（单位rpm）
		 * @param BwrSpeedLimit  正向速度限制（单位rpm）
		 * @return 0 成功；其它 失败
		 */
		[DllImport("NimServoSDK.dll")]
		public static extern int Nim_set_PT_SpeedLimit(uint hMaster, int nodeId, ushort FwrSpeedLimit, ushort BwrSpeedLimit);
		
		/**
		 * @brief 获取轮廓转矩模式下速度限制(2007:10h、2007:11h)
		 * @param hMaster 主站对象句柄
		 * @param nodeId 从站地址
		 * @param FwrSpeedLimit  正向速度限制（单位rpm）
		 * @param BwrSpeedLimit  正向速度限制（单位rpm）
		 * @return 0 成功；其它 失败
		 */
		[DllImport("NimServoSDK.dll")]
		public static extern int Nim_get_PT_SpeedLimit(uint hMaster, int nodeId, out ushort FwrSpeedLimit, out ushort BwrSpeedLimit);

		/**
		 * @brief 设置转矩斜坡
		 * @param hMaster 主站对象句柄
		 * @param nodeId 轴地址
		 * @param torqueRamp 转矩斜坡：0 无斜坡；>0 每秒钟增加的转矩值（单位：0.001倍额定转矩）
		 * @return 0 成功；其它 失败
		 */
		[DllImport("NimServoSDK.dll")]
		public static extern int Nim_set_PT_TorqueRamp(uint hMaster, int nodeId, uint torqueRamp);

		/**
		 * @brief 获取转矩斜坡
		 * @param hMaster 主站对象句柄
		 * @param nodeId 轴地址
		 * @param torqueRamp 转矩斜坡：0 无斜坡；>0 每秒钟增加的转矩值（单位：0.001倍额定转矩）
		 * @return 0 成功；其它 失败
		 */
		[DllImport("NimServoSDK.dll")]
		public static extern int Nim_get_PT_TorqueRamp(uint hMaster, int nodeId, out uint torqueRamp);

        /**
         * @brief 快速停止当前动作
         * @param nodeId 轴地址
         * @param bSDO 1 使用SDO控制；0 使用PDO
         * @return 0 成功；其它 失败
         */
        [DllImport("NimServoSDK.dll", EntryPoint = "Nim_fastStop")]
        public static extern int Nim_fastStop(uint hMaster, int nodeId, int bSDO);

        /**
         * @brief 清除轴故障
         * @param nodeId 轴地址
         * @param bSDO 1 使用SDO控制；0 使用PDO
         * @return 0 成功；其它 失败
         */
        [DllImport("NimServoSDK.dll", EntryPoint = "Nim_clearError")]
        public static extern int Nim_clearError(uint hMaster, int nodeId, int bSDO);

        /**
         * @brief 获取最新报警(603F)
         * @param nodeId 轴地址
         * @param alarmCode 输出参数，执行成功时返回报警码
         * @param bSDO 1 使用SDO控制；0 使用PDO
         * @return 0 成功；其它 失败
         */
        [DllImport("NimServoSDK.dll", EntryPoint = "Nim_get_newestAlarm")]
        public static extern int Nim_get_newestAlarm(uint hMaster, int nodeId, ref uint alarmCode, int bSDO);

        /**
         * @brief 获取历史报警数量(1003:00)
         * @param nodeId 轴地址
         * @param alarmCode 输出参数，执行成功时返回历史报警数量
         * @return 0 成功；其它 失败
         */
        [DllImport("NimServoSDK.dll", EntryPoint = "Nim_get_alarmCount")]
        public static extern int Nim_get_alarmCount(uint hMaster, int nodeId, ref int count);

        /**
         * @brief 获取历史报警(1003:01~10h)
         * @param nodeId 轴地址
         * @param index 历史报警序号,取值范围：1~16
         * @param alarmCode 输出参数，执行成功时返回报警码
         * @return 0 成功；其它 失败
         */
        [DllImport("NimServoSDK.dll", EntryPoint = "Nim_get_alarm")]
        public static extern int Nim_get_alarm(uint hMaster, int nodeId, int index, ref uint alarmCode);

        /**
         * @brief 获取轮廓速度(6081)
         * @param nodeId 轴地址
         * @param fVelocity 速度（用户单位/s）
         * @return 0 成功；其它 失败
         */
        [DllImport("NimServoSDK.dll", EntryPoint = "Nim_get_profileVelocity")]
        public static extern int Nim_get_profileVelocity(uint hMaster, int nodeId, ref double fVelocity);

        /**
         * @brief 获取轮廓加速度(6083)
         * @param nodeId 轴地址
         * @param accel 加速度（用户单位/s^2）
         * @return 0 成功；其它 失败
         */
        [DllImport("NimServoSDK.dll", EntryPoint = "Nim_get_profileAccel")]
        public static extern int Nim_get_profileAccel(uint hMaster, int nodeId, ref double accel);

        /**
         * @brief 获取轮廓减速度(6084)
         * @param nodeId 轴地址
         * @param decel 减速度（用户单位/s^2）
         * @return 0 成功；其它 失败
         */
        [DllImport("NimServoSDK.dll", EntryPoint = "Nim_get_profileDecel")]
        public static extern int Nim_get_profileDecel(uint hMaster, int nodeId, ref double decel);

        /**
         * @brief 获取快速停机减速度(6085)
         * @param nodeId 轴地址
         * @param decel 减速度（用户单位/s^2）
         * @return 0 成功；其它 失败
         */
        [DllImport("NimServoSDK.dll", EntryPoint = "Nim_get_quickStopDecel")]
        public static extern int Nim_get_quickStopDecel(uint hMaster, int nodeId, ref double decel);

        /**
         * @brief 设置轮廓速度(6081)
         * @param nodeId 轴地址
         * @param fVelocity 速度（用户单位/s）
         * @return 0 成功；其它 失败
         */
        [DllImport("NimServoSDK.dll", EntryPoint = "Nim_set_profileVelocity")]
        public static extern int Nim_set_profileVelocity(uint hMaster, int nodeId, double fVelocity);

        /**
         * @brief 设置轮廓加速度(6083)
         * @param nodeId 轴地址
         * @param accel 加速度（用户单位/s^2）
         * @return 0 成功；其它 失败
         */
        [DllImport("NimServoSDK.dll", EntryPoint = "Nim_set_profileAccel")]
        public static extern int Nim_set_profileAccel(uint hMaster, int nodeId, double accel);

        /**
         * @brief 设置轮廓减速度(6084)
         * @param nodeId 轴地址
         * @param decel 减速度（用户单位/s^2）
         * @return 0 成功；其它 失败
         */
        [DllImport("NimServoSDK.dll", EntryPoint = "Nim_set_profileDecel")]
        public static extern int Nim_set_profileDecel(uint hMaster, int nodeId, double decel);

        /**
         * @brief 设置快速停机减速度(6085)
         * @param nodeId 轴地址
         * @param decel 减速度（用户单位/s^2）
         * @return 0 成功；其它 失败
         */
        [DllImport("NimServoSDK.dll", EntryPoint = "Nim_set_quickStopDecel")]
        public static extern int Nim_set_quickStopDecel(uint hMaster, int nodeId, double decel);
        
        /**
         * @brief 获取原点偏移
         * @param nodeId 轴地址
         * @param offset 原点偏移，用户单位
         * @return 0 成功；其它 失败
         */
        [DllImport("NimServoSDK.dll", EntryPoint = "Nim_get_homeOffset")]
        public static extern int Nim_get_homeOffset(uint hMaster, int nodeId, ref double offset);

        /**
         * @brief 获取原点回归速度(6099:01/02)
         * @param nodeId 轴地址
         * @param velocity1 寻找开关速度（用户单位/s）
         * @param velocity2 寻找原点速度（用户单位/s）
         * @return 0 成功；其它 失败
         */
        [DllImport("NimServoSDK.dll", EntryPoint = "Nim_get_goHome_velocity")]
        public static extern int Nim_get_goHome_velocity(uint hMaster, int nodeId, ref double velocity1, ref double velocity2);

        /**
         * @brief 获取原点回归加速度(609A)
         * @param nodeId 轴地址
         * @param accel 加速度（用户单位/s^2）
         * @return 0 成功；其它 失败
         */
        [DllImport("NimServoSDK.dll", EntryPoint = "Nim_get_goHome_accel")]
        public static extern int Nim_get_goHome_accel(uint hMaster, int nodeId, ref double accel);

        /**
         * @brief 设置原点偏移
         * @param nodeId 轴地址
         * @param offset 原点偏移，用户单位
         * @return 0 成功；其它 失败
         */
        [DllImport("NimServoSDK.dll", EntryPoint = "Nim_set_homeOffset")]
        public static extern int Nim_set_homeOffset(uint hMaster, int nodeId, double offset);

        /**
         * @brief 设置原点回归速度(6099:01/02)
         * @param nodeId 轴地址
         * @param velocity1 寻找开关速度（用户单位/s）
         * @param velocity2 寻找原点速度（用户单位/s）
         * @return 0 成功；其它 失败
         */
        [DllImport("NimServoSDK.dll", EntryPoint = "Nim_set_goHome_velocity")]
        public static extern int Nim_set_goHome_velocity(uint hMaster, int nodeId, double velocity1, double velocity2);

        /**
         * @brief 设置原点回归加速度(609A)
         * @param nodeId 轴地址
         * @param accel 加速度（用户单位/s^2）
         * @return 0 成功；其它 失败
         */
        [DllImport("NimServoSDK.dll", EntryPoint = "Nim_set_goHome_accel")]
        public static extern int Nim_set_goHome_accel(uint hMaster, int nodeId, double accel);

        /**
         * @brief 获取VM模式下的加速度
         * @param nodeId 从站地址
         * @param accel 输出参数，成功时返回加速度（rpm/s）
         * @return 0 成功；其它 失败
         */
        [DllImport("NimServoSDK.dll", EntryPoint = "Nim_get_vmAccel")]
        public static extern int Nim_get_vmAccel(uint hMaster, int nodeId, ref double accel);

        /**
         * @brief 设置VM模式下的加速度
         * @param nodeId 从站地址
         * @param deltaV 速度变化量（rpm）
         * @param deltaT 时间变化量（S）
         *        加速度 = deltaV/deltaT
         * @return 0 成功；其它 失败
         */
        [DllImport("NimServoSDK.dll", EntryPoint = "Nim_set_vmAccel")]
        public static extern int Nim_set_vmAccel(uint hMaster, int nodeId, uint deltaV, uint deltaT);

        /**
         * @brief 获取VM模式下的减速度
         * @param nodeId 从站地址
         * @param accel 输出参数，成功时返回减速度（rpm/s）
         * @return 0 成功；其它 失败
         */
        [DllImport("NimServoSDK.dll", EntryPoint = "Nim_get_vmDecel")]
        public static extern int Nim_get_vmDecel(uint hMaster, int nodeId, ref double decel);

        /**
         * @brief 设置VM模式下的减速度
         * @param nodeId 从站地址
         * @param deltaV 速度变化量（rpm）
         * @param deltaT 时间变化量（S）
         *        减速度 = deltaV/deltaT
         * @return 0 成功；其它 失败
         */
        [DllImport("NimServoSDK.dll", EntryPoint = "Nim_set_vmDecel")]
        public static extern int Nim_set_vmDecel(uint hMaster, int nodeId, uint deltaV, uint deltaT);

        /**
         * @brief 通过6069获取当前速度
         * @param nodeId 轴地址
         * @param velocity 输出参数，执行成功时返回当前速度（用户单位/s）
         * @param bSDO 1 使用SDO输出；0 使用PDO
         * @return 0 成功；其它 失败
         */
        [DllImport("NimServoSDK.dll", EntryPoint = "Nim_get_currentVelocity")]
        public static extern int Nim_get_currentVelocity(uint hMaster, int nodeId, ref double velocity, int bSDO);

        /**
         * @brief 通过606C获取当前速度
         * @param nodeId 轴地址
         * @param velocity 输出参数，执行成功时返回当前速度（用户单位/s）
         * @param bSDO 1 使用SDO输出；0 使用PDO
         * @return 0 成功；其它 失败
         */
        [DllImport("NimServoSDK.dll", EntryPoint = "Nim_get_currentVelocity2")]
        public static extern int Nim_get_currentVelocity2(uint hMaster, int nodeId, ref double velocity, int bSDO);

        /**
         * @brief 通过606C获取当前电机速度
         * @param nodeId 轴地址
         * @param speed 输出参数，执行成功时返回当前速度（rpm）
         * @param bSDO 1 使用SDO输出；0 使用PDO
         * @return 0 成功；其它 失败
         */
        [DllImport("NimServoSDK.dll", EntryPoint = "Nim_get_currentMotorSpeed")]
        public static extern int Nim_get_currentMotorSpeed(uint hMaster, int nodeId, ref int speed, int bSDO);

        /**
         * @brief 通过6064获取当前位置
         * @param nodeId 轴地址
         * @param position 输出参数，执行成功时返回当前位置
         * @param bSDO 1 使用SDO输出；0 使用PDO
         * @return 0 成功；其它 失败
         */
        [DllImport("NimServoSDK.dll", EntryPoint = "Nim_get_currentPosition")]
        public static extern int Nim_get_currentPosition(uint hMaster, int nodeId, ref double position, int bSDO);

        /**
         * @brief 获取位置限制值(607D:01/02)
         * @param nodeId 轴地址
         * @param minPos 输出参数，执行成功时返回最小极限位置
         * @param maxPos 输出参数，执行成功时返回最大极限位置
         * @return 0 成功；其它 失败
         */
        [DllImport("NimServoSDK.dll", EntryPoint = "Nim_get_posLimit")]
        public static extern int Nim_get_posLimit(uint hMaster, int nodeId, ref double minPos, ref double maxPos);

        /**
         * @brief 设置位置限制值(607D:01/02)
         * @param nodeId 轴地址
         * @param pos 输出参数，执行成功时返回最大极限位置
         * @return 0 成功；其它 失败
         */
        [DllImport("NimServoSDK.dll", EntryPoint = "Nim_set_posLimit")]
        public static extern int Nim_set_posLimit(uint hMaster, int nodeId, double minPos, double maxPos);

        /**
         * @brief 设置最大速度(607F)
         * @param nodeId 轴地址
         * @param velocity 最大速度（用户单位/s）
         * @return 0 成功；其它 失败
         */
        [DllImport("NimServoSDK.dll", EntryPoint = "Nim_set_maxVelocity")]
        public static extern int Nim_set_maxVelocity(uint hMaster, int nodeId, double velocity);

        /**
         * @brief 获取最大速度
         * @param nodeId 轴地址
         * @param velocity 输出参数，成功时返回最大速度（用户单位/s）
         * @return 0 成功；其它 失败
         */
        [DllImport("NimServoSDK.dll", EntryPoint = "Nim_get_maxVelocity")]
        public static extern int Nim_get_maxVelocity(uint hMaster, int nodeId, ref double velocity);

        /**
         * @brief 获取最大电机速度(6080)
         * @param nodeId 从站地址
         * @param torque 输出参数，成功时返回最大电机速度（rpm）
         * @return 0 成功；其它 失败
         */
        [DllImport("NimServoSDK.dll", EntryPoint = "Nim_get_maxMotorSpeed")]
        public static extern int Nim_get_maxMotorSpeed(uint hMaster, int nodeId, ref uint speed);

        /**
         * @brief 设置最大电机速度(6080)
         * @param nodeId 从站地址
         * @param torque  最大电机速度（rpm）
         * @return 0 成功；其它 失败
         */
        [DllImport("NimServoSDK.dll", EntryPoint = "Nim_set_maxMotorSpeed")]
        public static extern int Nim_set_maxMotorSpeed(uint hMaster, int nodeId, uint speed);

        /**
         * @brief 设置最大转矩(6072)
         * @param nodeId 从站地址
         * @param torque  最大转矩（0.001倍额定转矩）
         * @return 0 成功；其它 失败
         */
        [DllImport("NimServoSDK.dll", EntryPoint = "Nim_set_maxTorque")]
        public static extern int Nim_set_maxTorque(uint hMaster, int nodeId, uint torque);

        /**
         * @brief 获取最大转矩(6072)
         * @param nodeId 从站地址
         * @param torque 输出参数，成功时返回最大转矩（0.001倍额定转矩）
         * @return 0 成功；其它 失败
         */
        [DllImport("NimServoSDK.dll", EntryPoint = "Nim_get_maxTorque")]
        public static extern int Nim_get_maxTorque(uint hMaster, int nodeId, ref uint torque);
        
        /**
         * @brief 获取VM模式下的速度限制值
         * @param nodeId 从站地址
         * @param minSpeed 最小速度（rpm）
         * @param maxSpeed 最大速度（rpm）
         * @return 0 成功；其它 失败
         */
        [DllImport("NimServoSDK.dll", EntryPoint = "Nim_get_vmSpeedLimit")]
        public static extern int Nim_get_vmSpeedLimit(uint hMaster, int nodeId, ref uint minSpeed, ref uint maxSpeed);
        
        /**
         * @brief 设置VM模式下的速度限制值
         * @param nodeId 从站地址
         * @param minSpeed 最小速度（rpm）
         * @param maxSpeed 最大速度（rpm）
         * @return 0 成功；其它 失败
         */
        [DllImport("NimServoSDK.dll", EntryPoint = "Nim_set_vmSpeedLimit")]
        public static extern int Nim_set_vmSpeedLimit(uint hMaster, int nodeId, uint minSpeed, uint maxSpeed);

        /**
        * @brief 设置用户单位的转换系数
        * @param nodeId 轴地址
        * @param factor 转换系数（电机编码器单位/用户单位）
        * @return 0 成功；其它 失败
        */
        [DllImport("NimServoSDK.dll", EntryPoint = "Nim_set_unitsFactor")]
        public static extern int Nim_set_unitsFactor(uint hMaster, int nodeId, double factor);

        /**
         * @brief 获取用户单位的转换系数
         * @param nodeId 轴地址
         * @param factor 转换系数（电机编码器单位/用户单位）
         * @return 0 成功；其它 失败
         */
        [DllImport("NimServoSDK.dll", EntryPoint = "Nim_get_unitsFactor")]
        public static extern int Nim_get_unitsFactor(uint hMaster, int nodeId, ref double factor);

        /**
         * @brief  设置DO输出
         * @param nodeId 轴地址
         * @param nDOs DO值：每位表示一路DO：bit0表示DO1；bit1表示DO2，以此类推
         *                  1 输出高电平；0 输出低电平
         * @param bSDO 1 使用SDO输出；0 使用PDO
         * @return 0 成功；其它 失败
         */
        [DllImport("NimServoSDK.dll", EntryPoint = "Nim_set_DOs")]
        public static extern int Nim_set_DOs(uint hMaster, int nodeId, uint nDOs, int bSDO);

        /**
         * @brief  读取DI输入
         * @param nodeId 轴地址
         * @param nDIs DI值：每位表示一路DI：bit0表示DI1；bit1表示DI2，以此类推
         *                  1 输入高电平；0 输入低电平
         * @param bSDO 1 使用SDO读取；0 使用PDO
         * @return 0 成功；其它 失败
         */
        [DllImport("NimServoSDK.dll", EntryPoint = "Nim_get_DIs")]
        public static extern int Nim_get_DIs(uint hMaster, int nodeId, ref uint nDIs, int bSDO);

        /**
         * @brief  设置VDI值
         * @param nodeId 轴地址
         * @param nVDIs VDI值：每位表示一路VDI：bit0表示VDI1；bit1表示VDI2，以此类推
         *                  1 输出高电平；0 输出低电平
         * @return 0 成功；其它 失败
         */
        [DllImport("NimServoSDK.dll", EntryPoint = "Nim_set_VDIs")]
        public static extern int Nim_set_VDIs(uint hMaster, int nodeId, uint nVDIs);
        
        /**
         * @brief 保存所有参数到设备
         * @param nodeId 从站地址
         * @param timeoutMS 超时时间，单位：ms
         * @return 0 成功；其它 失败
         */
        [DllImport("NimServoSDK.dll", EntryPoint = "Nim_save_AllParams")]
        public static extern int Nim_save_AllParams(uint hMaster, int nodeId, int timeoutMS);
    }
}
