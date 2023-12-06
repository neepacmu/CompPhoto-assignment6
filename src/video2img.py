import cv2

def save_frames_from_video(video_path, start_time, end_time, output_folder):
    # Open the video file
    video_capture = cv2.VideoCapture(video_path)

    # Get the frame rate of the video
    fps = video_capture.get(cv2.CAP_PROP_FPS)

    # Calculate frame indices for start and end times
    start_frame = int(start_time * fps)
    end_frame = int(end_time * fps)

    # Set the frame position to the start frame
    video_capture.set(cv2.CAP_PROP_POS_FRAMES, start_frame)

    # Loop through the frames and save images
    current_frame = start_frame
    temp = 0
    while current_frame < end_frame:

            
        ret, frame = video_capture.read()
        if not ret:
            break
        if temp % 3 == 0:
        
            # Save the frame as an image
            cv2.imwrite(f"{output_folder}/frame_{current_frame}.jpg", frame)
        
        temp += 1
        current_frame += 1

    # Release the video capture object
    video_capture.release()

# Example usage
video_path = 'DSC_0560.MOV'
start_time_seconds = 48  # Start time in seconds
end_time_seconds = 60  # End time in seconds
output_folder = 'custom/box9'

save_frames_from_video(video_path, start_time_seconds, end_time_seconds, output_folder)