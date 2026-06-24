using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Threading;
using NimServoSDK_DLL;

namespace NimServoSDK_Test
{
    class Program
    {
        private  void MainTest(string[] args)
        {
            if (args.Length < 2)
            {
                Console.WriteLine("Usage: NimServoSDK_Test2 commType commParam [unitFactor]");
                return;
            }

            NimServoSDK.Nim_setLogFlags(1);
            int ret = NimServoSDK.Nim_getLogFlags();
            Console.WriteLine(string.Format("ret: {0}", ret));

            int nCommType = int.Parse(args[0]);
            if (0 != NimServoSDK.Nim_init(""))
            {
                Console.WriteLine("exec Nim_init faild");
            }
            uint hMaster = 0;
            if (0 != NimServoSDK.Nim_create_master(nCommType, ref hMaster))
            {
                Console.WriteLine("exec Nim_create_master faild\n");
            }
            if (nCommType < 0 || nCommType > 2)
                return;

            string conn_str = "";
            switch (nCommType)
            {
                case 0:     //CANopen
                    int nDevType = int.Parse(args[1]);
                    switch (nDevType)
                    {
                        case 1001:      //NiMotion USB-CAN Device
                            conn_str = "{\"DevType\": \"1001\", \"DevIndex\": 0, \"Baudrate\": 8,"
                                        + "\"PDOIntervalMS\": 10, \"SyncIntervalMS\": 10}";
                            break;
                        case 1002:      //Zhoulg USB-CAN Device
                            conn_str = "{\"DevType\": \"1002\", \"DevSubType\": 3, \"DevIndex\": 0, \"ChannelIndex\": 0,"
                                        + " \"Baudrate\": 8, \"PDOIntervalMS\": 10, \"SyncIntervalMS\": 10}";
                            break;
                        case 1003:      //Ixxat USB-CAN Device
                            conn_str = "{\"DevType\": \"%s\", \"DevIndex\": 0, \"Baudrate\": 8,"
                                        + " \"PDOIntervalMS\": 10, \"SyncIntervalMS\": 10}";
                            break;
                        case 1004:      //NiMotion TCP-CAN Device
                            conn_str = "{\"DevType\": \"1004\", \"IP\": \"192.168.0.96\", \"Port\": 40001,"
                                        + " \"Baudrate\": 8, \"PDOIntervalMS\": 10, \"SyncIntervalMS\": 10}";
                            break;
                        case 1005:      //Linux SocketCAN Device
                            conn_str = "{\"DevType\": \"%s\", \"DeviceName\": \"can0\","
                                        + " \"Baudrate\": 8, \"PDOIntervalMS\": 10, \"SyncIntervalMS\": 10}";
                            break;
                        default:
                            conn_str = "{\"DevType\": \"1001\", \"DevIndex\": 0, \"Baudrate\": 8,"
                                        + "\"PDOIntervalMS\": 10, \"SyncIntervalMS\": 10}";
                            break;
                    }
                    break;
                case 1:     //EtherCAT
                    conn_str = "{\"NetworkAdapter\": \"" + args[1] + "\", \"OverlappingPDO\": true, \"PDOIntervalMS\": 5}";
                    break;
                case 2:     //Modbus
                    conn_str = "{\"SerialPort\": \"" + args[1] + "\", \"Baudrate\": 115200," 
                               + " \"Parity\": \"N\", \"DataBits\": 8, \"StopBits\": 1,"
                               + " \"PDOIntervalMS\": 20, \"SyncIntervalMS\": 0}";
                    break;
                default:
                    break;
            }
            Console.WriteLine(conn_str);

            if (0 != NimServoSDK.Nim_master_run(hMaster, conn_str))
            {
                Console.WriteLine("exec Nim_master_run faild");
                return;
            }
			NimServoSDK.Nim_master_changeToPreOP(hMaster);
            Thread.Sleep(50);		// 必要延时(Nim_master_changeToPreOP后)

            // 扫描电机
            int nAddr = 0;
            NimServoSDK.Nim_scan_nodes(hMaster, 1, 10);
            for (int i = 0; i < 10; i++)
            {
                if (0 != NimServoSDK.Nim_is_online(hMaster, i))
                {
                    nAddr = i;
                    Console.WriteLine(string.Format("motor {0} is online", i));
                    break;
                }
            }
            if (nAddr == 0)
            {
                Console.WriteLine("There is no motor online");
                NimServoSDK.Nim_master_stop(hMaster);
                return;
            }

            // 设置传动比
            double fUnitFactor = 10000.0;
            if (args.Length > 3)
            {
                fUnitFactor = double.Parse(args[2]);
            }
            Console.WriteLine(string.Format("UnitFactor = {0}", fUnitFactor));

            // 加载电机参数表
            int nRe = 0;
            if (nCommType == 2)
                nRe = NimServoSDK.Nim_load_params(hMaster, nAddr, "Modbus.db");
            else if (nCommType == 1)
                nRe = NimServoSDK.Nim_load_params(hMaster, nAddr, "EtherCAT.db");
			else
				nRe = NimServoSDK.Nim_load_params(hMaster, nAddr, "CANopen.db");
			
            if (0 != nRe)
            {
                Console.WriteLine("exec Nim_load_params failed");
                return;
            }

            // 读取电机PDO配置
            nRe = NimServoSDK.Nim_read_PDOConfig(hMaster, nAddr);
            if (0 != nRe)
            {
                Console.WriteLine("exec Nim_read_PDOConfig failed\r");
                return;
            }

            // 主站切换到OP模式
            nRe = NimServoSDK.Nim_master_changeToOP(hMaster);
			Thread.Sleep(50);		// 必要延时(Nim_master_changeToOP后)
            nRe = NimServoSDK.Nim_set_unitsFactor(hMaster, nAddr, fUnitFactor);
			nRe = NimServoSDK.Nim_clearError(hMaster, nAddr, 1);
            // 设置操作模式并抱机
            nRe = NimServoSDK.Nim_power_off(hMaster, nAddr, 1);
            Thread.Sleep(50);      // 必要延时(Nim_power_off后)
            nRe = NimServoSDK.Nim_set_workMode(hMaster, nAddr, 1, 1);
            Thread.Sleep(50);      // 必要延时(Nim_set_workMode后)		
            nRe = NimServoSDK.Nim_set_profileVelocity(hMaster, nAddr, 15.0); 
            nRe = NimServoSDK.Nim_set_profileAccel(hMaster, nAddr, 12.5);
            nRe = NimServoSDK.Nim_set_profileDecel(hMaster, nAddr, 12.5);
            nRe = NimServoSDK.Nim_power_on(hMaster, nAddr, 1);
            Thread.Sleep(200);       // 必要延时(Nim_power_on后)

            Console.WriteLine("********************move relative*********************");
            nRe = NimServoSDK.Nim_moveRelative(hMaster, nAddr, 10.0, 0, 1);
            double fPos = 0.0;
            UInt16 statusWord = 0;
            do
            {
                Thread.Sleep(50);
                if (0 == NimServoSDK.Nim_get_currentPosition(hMaster, nAddr, ref fPos, 0)
                    && 0 == NimServoSDK.Nim_get_statusWord(hMaster, nAddr, ref statusWord, 0))
                    Console.Write(string.Format("statusword = {0:x4} \t currentpos = {1}\t\r", statusWord, fPos));
            } while ((statusWord & 0x400) == 0);
            Console.WriteLine("");

            Thread.Sleep(200);
            Console.WriteLine("********************move absolute*********************");
            nRe = NimServoSDK.Nim_moveAbsolute(hMaster, nAddr, 0.0, 0, 1);
            do
            {
                Thread.Sleep(50);
                if (0 == NimServoSDK.Nim_get_currentPosition(hMaster, nAddr, ref fPos, 0)
                        && 0 == NimServoSDK.Nim_get_statusWord(hMaster, nAddr, ref statusWord, 0))
                    Console.Write(string.Format("statusword = {0:x4} \t currentpos = {1}\t\r", statusWord, fPos));
            }while ((statusWord & 0x400) == 0);
            Console.WriteLine("");

            nRe = NimServoSDK.Nim_power_off(hMaster, nAddr, 1);
            Thread.Sleep(50);		// 必要延时(Nim_power_off后)
            nRe = NimServoSDK.Nim_master_changeToPreOP(hMaster);
			Thread.Sleep(50);		// 必要延时(Nim_master_changeToPreOP后)
            nRe = NimServoSDK.Nim_master_stop(hMaster);
            NimServoSDK.Nim_destroy_master(hMaster);
            NimServoSDK.Nim_clean();
        }
    }
}
