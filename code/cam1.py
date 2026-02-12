import cv2
import time
import threading
import queue

class USBCamera:
    def __init__(self, camera_index=0, width=1920, height=1080, fps=30):
        """
        USB相机类，使用多线程提高帧率
        
        Args:
            camera_index: 相机索引
            width: 图像宽度
            height: 图像高度
            fps: 目标帧率
        """
        self.camera_index = camera_index
        self.width = width
        self.height = height
        self.fps = fps
        self.cap = None
        self.frame_queue = queue.Queue(maxsize=2)  # 限制队列大小以减少延迟
        self.running = False
        self.thread = None
        
    def initialize(self):
        """初始化相机"""
        self.cap = cv2.VideoCapture(self.camera_index)
        
        if not self.cap.isOpened():
            raise Exception(f"无法打开相机 {self.camera_index}")
        
        # 设置MJPEG格式
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        
        # 设置分辨率和帧率
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        
        # 关闭自动曝光
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0)
        
        # 设置缓冲区大小
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        
        # 验证实际设置的参数
        actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
        print(f"相机配置结果：")
        print(f"分辨率：{actual_width}×{actual_height}")
        print(f"设置帧率：{actual_fps}fps")
        
    def capture_frames(self):
        """捕获帧的线程函数"""
        while self.running:
            ret, frame = self.cap.read()
            if ret:
                # 如果队列已满，丢弃最旧的帧
                if self.frame_queue.full():
                    try:
                        self.frame_queue.get_nowait()
                    except queue.Empty:
                        pass
                
                # 将新帧放入队列
                self.frame_queue.put(frame)
    
    def start(self):
        """启动相机"""
        self.initialize()
        self.running = True
        self.thread = threading.Thread(target=self.capture_frames)
        self.thread.daemon = True
        self.thread.start()
    
    def get_frame(self):
        """获取最新帧"""
        try:
            return self.frame_queue.get_nowait()
        except queue.Empty:
            return None
    
    def stop(self):
        """停止相机"""
        self.running = False
        if self.thread:
            self.thread.join()
        if self.cap:
            self.cap.release()

def open_usb_camera_1080p_30fps(camera_index=0):
    """
    打开USB相机并配置为1920×1080 30fps
    
    Args:
        camera_index: 相机索引，默认0（多个相机可尝试1、2等）
    """
    camera = USBCamera(camera_index, 1920, 1080, 30)
    
    try:
        camera.start()
        
        # 计算实际帧率
        frame_count = 0
        start_time = time.time()
        fps_display = 0
        
        print("按 'q' 退出程序...")
        while True:
            frame = camera.get_frame()
            if frame is not None:
                # 计算实时帧率
                frame_count += 1
                elapsed_time = time.time() - start_time
                if elapsed_time >= 1.0:
                    fps_display = frame_count / elapsed_time
                    frame_count = 0
                    start_time = time.time()
                
                # 在画面上显示帧率
                cv2.putText(frame, f"FPS: {fps_display:.1f}", (10, 30), 
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                
                # 显示画面
                cv2.imshow("USB Camera 1080p 30fps", frame)
                
                # 按q退出
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
    except Exception as e:
        print(f"错误: {e}")
    finally:
        camera.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    # 调用函数，使用索引4的相机（与您的ffplay命令一致）
    open_usb_camera_1080p_30fps(camera_index=4)
