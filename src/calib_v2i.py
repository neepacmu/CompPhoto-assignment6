import cv2

def save_frames(video_path, timestamps, output_folder):
    cap = cv2.VideoCapture(video_path)
    
    
    frame_rate = cap.get(cv2.CAP_PROP_FPS)
    total_frames = cap.get(cv2.CAP_PROP_FRAME_COUNT)
    
    for timestamp in timestamps:
        
        cap.set(cv2.CAP_PROP_POS_MSEC, timestamp * 1000)
        success, frame = cap.read()
        
        output_file = f"{output_folder}/frame_{timestamp}.jpg"
        cv2.imwrite(output_file, frame)
        print(f"Frame at {timestamp} seconds saved as {output_file}")
    
    cap.release()

video_file = 'DSC_0559.MOV'
time_stamps = [20, 23, 27, 29, 33, 35, 39, 43]  # Timestamps in seconds
output_folder = './custom/calib3'

save_frames(video_file, time_stamps, output_folder)
