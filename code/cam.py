import cv2
import argparse

def show_camera(device_id=0, width=640, height=480, fps=15):
    """
    显示USB摄像头画面并输出相机支持的参数
    :param device_id: 摄像头设备号，默认0（/dev/video0）
    :param width: 画面宽度，默认640
    :param height: 画面高度，默认480
    :param fps: 帧率限制，默认15（降低资源占用）
    """
    # 打开摄像头
    cap = cv2.VideoCapture(device_id)
    if not cap.isOpened():
        raise Exception(f"无法打开摄像头设备 /dev/video{device_id}")
    
    # 查询并输出相机支持的参数
    print("=== 相机支持的参数 ===")
    print(f"最大分辨率: {int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}")
    print(f"最大帧率: {cap.get(cv2.CAP_PROP_FPS)}")
    print(f"支持的格式: {cap.get(cv2.CAP_PROP_FORMAT)}")
    print(f"亮度范围: {cap.get(cv2.CAP_PROP_BRIGHTNESS)}")
    print(f"对比度范围: {cap.get(cv2.CAP_PROP_CONTRAST)}")
    print(f"饱和度范围: {cap.get(cv2.CAP_PROP_SATURATION)}")
    print(f"色调范围: {cap.get(cv2.CAP_PROP_HUE)}")
    print(f"增益范围: {cap.get(cv2.CAP_PROP_GAIN)}")
    print(f"曝光范围: {cap.get(cv2.CAP_PROP_EXPOSURE)}")
    print(f"白平衡范围: {cap.get(cv2.CAP_PROP_WHITE_BALANCE_BLUE_U)}")
    print(f"自动曝光: {cap.get(cv2.CAP_PROP_AUTO_EXPOSURE)}")
    print(f"自动白平衡: {cap.get(cv2.CAP_PROP_AUTO_WB)}")
    # 移除了不兼容的CAP_PROP_AUTOGRAB
    
    # 设置用户指定的参数
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)
    
    # 输出实际设置的参数
    print("\n=== 实际设置的参数 ===")
    print(f"分辨率: {int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}")
    print(f"帧率: {cap.get(cv2.CAP_PROP_FPS)}")
    
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("无法获取帧数据")
                break
            
            # 显示帧
            cv2.imshow('Camera', frame)
            
            # 按'q'键退出
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        # 释放资源
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    # 命令行参数解析
    parser = argparse.ArgumentParser(description="Python USB摄像头查看器")
    parser.add_argument("--device", type=int, default=4, help="摄像头设备号，默认0")
    # parser.add_argument("--width", type=int, default=640, help="画面宽度，默认640")
    # parser.add_argument("--height", type=int, default=480, help="画面高度，默认480")
    parser.add_argument("--width", type=int, default=int(1280/8*6), help="画面宽度，默认640")
    parser.add_argument("--height", type=int, default=int(760/8*6), help="画面高度，默认480")
    parser.add_argument("--fps", type=int, default=30, help="帧率，默认15")
    args = parser.parse_args()

    # 启动摄像头显示
    try:
        show_camera(args.device, args.width, args.height, args.fps)
    except Exception as e:
        print(f"错误: {e}")
        print("提示：请检查摄像头是否连接、是否被其他程序占用，或执行 sudo usermod -aG video $USER 并重启")
