using System.Threading;
using System.Threading.Tasks;

namespace WpfRobot.joint7
{
    public interface ISeventhAxisController
    {
        bool IsConnected { get; }

        Task MoveToDegAsync(
            double q7Deg,
            double velocityDegPerSec,
            double accelerationDegPerSec2,
            CancellationToken token);

        Task StopAsync();
    }

    /// <summary>
    /// 空实现：第七轴控制程序还没写好之前先用它。
    /// 不会报错，也不会影响 UR 和 VTK。
    /// </summary>
    public class NullSeventhAxisController : ISeventhAxisController
    {
        public bool IsConnected => false;

        public Task MoveToDegAsync(
            double q7Deg,
            double velocityDegPerSec,
            double accelerationDegPerSec2,
            CancellationToken token)
        {
            return Task.CompletedTask;
        }

        public Task StopAsync()
        {
            return Task.CompletedTask;
        }
    }
}