import os
import cv2
from cp_hw6 import pixel2ray, set_axes_equal
import matplotlib.pyplot as plt
import numpy as np
from tqdm import tqdm
from skimage.color import rgb2gray


def read_files(data_dir):
    
    image_list = []
    image_path = []

    for file_n in sorted(os.listdir(data_dir)):
        
        if '.jpg'  in file_n:
            #print(file_n)
            file_name = os.path.join(data_dir, file_n)

            image = cv2.imread(file_name)
            #print(image.shape)

            # h,w = image.shape[0], image.shape[1]
            # n_ = 4
            
            # #color_img = cv2.resize(color_img, (1500, 1000))

            # image = cv2.resize(image, (int(w/n_), int(h/n_)))

            # cv2.imwrite('temp.jpg', image)
            if image.shape[0] != 1080:
                continue
                print("WOW", image.shape)

            image = rgb2gray(image)

            #plt.imshow(image)
            #plt.show()


            image_list.append(image)

            

            image_path.append(file_name)

    #print(image_list)
    return np.array(image_list), image_path



def get_shadow_line(x_min, x_max, y_min, y_max, I_delta, img_list, viz = False):


    ans = []
    output = []

    for t in range(I_delta.shape[0]):

        #signs = np.sign(I_delta[t])

        #shifted_signs = np.roll(signs, shift=1, axis=1)

        #zero_crossings = np.where((signs * shifted_signs) < 0)
        zero_crossings = []
        A = []
        for y in range(y_min, y_max):

            for x in range(x_min, x_max):
            
                if I_delta[t,y,x-1] < 0 and I_delta[t,y,x] >= 0:
                    #zero_crossings.append([y, x-1])
                    A.append([x-1,y, 1])
                    #break

        

        
        if len(A) > 0:
            #print(t)
            
            A = np.array(A)
            #print(A.shape)
            U, S, V = np.linalg.svd(A)

            a,b,c = V[-1]

            output.append([a,b,c,t])

            if viz:
                print(a,b,c,t)
                t_str = f'0{str(t)}' if t < 100 else str(t)
                
                image = cv2.imread(img_list[t])

                
                
                x1 = int((-c - y_min*b)/a)
                x2 = int((-c - y_max*b)/a)

                
                color = (255, 0, 0)  # Blue color in BGR format
                thickness = 2  # Line thickness
                print(x1, x2, y_min, y_max)
                cv2.line(image, (x1, y_min), (x2, y_max), color, thickness)
                #print(y, t,  "LLL")

            
                cv2.imwrite(f'out_lines/temp_{str(t)}.jpg', image)
            
                # #break

    return np.array(output)


def get_per_pixel_shadow(I_delta):

    

    I_out = np.zeros_like(I_delta[0])

    for i in range(I_delta.shape[1]):
        for j in range(I_delta.shape[2]):

            signs = np.sign(I_delta[:, i, j])

            

            shifted_signs = np.roll(signs, shift=1, axis=0)

            #print(signs.shape, shifted_signs.shape, (signs * shifted_signs).shape)

            zero_crossings = np.where((signs * shifted_signs) < 0)


            idx = zero_crossings[0]
            if idx.any():
                #print(idx, zero_crossings)
                I_out[i][j] = idx[0]

    return I_out


def plane_from_points(p1, p2, p3, p4):
    # Define the plane passing through the four points
    v1 = p2 - p1
    v2 = p3 - p1
    normal = np.cross(v1, v2)
    d = -np.dot(normal, p1)
    #print(normal, p1, "HAHAH")
    return normal, d

def plane_from_points_four(p1, p2, p3, p4):
    # Define the plane passing through the four points
    v1 = p2 - p1
    v2 = p3 - p4
    normal = np.cross(v1, v2)
    d = -np.dot(normal, p1)
    #print(normal, p1, "HAHAH")
    return normal, d

def intersection_point(plane_normal, plane_d, ray_origin, ray_direction):
    # Compute intersection point between the plane and the ray
    denom = np.dot(plane_normal, ray_direction)
    
    if abs(denom) > 1e-6:  # Ensure the ray is not parallel to the plane
        t = -(np.dot(plane_normal, ray_origin) + plane_d) / denom
        #print("ADKASDA ", t)
        intersection = ray_origin + t * ray_direction
        #print("F : ", t)
        return intersection
    else:
        return None  # No intersection or parallel ray

    

def camera2world(p, R, T):
    #print(" FFFFF " ,p, T, (p - T[:,0]).T)
    P_plane = np.dot(R, (p - T[:,0]).T)
    return P_plane.T

def world2camera(p, R, T):
    #print(p.shape, T.shape, np.dot(R, p.T).shape, "LL")
    P_plane = np.dot(R, p.T) + T[:,0]
    return P_plane.T

def get_plane_intersections(output_lines, intrinsics, R, Tr, y_min, y_max):

    output_points1 = []
    output_points2 = []

    output = {}

    for line in output_lines:
        
        a,b,c,t = line
        x1 = int((-c - y_min*b)/a)
        x2 = int((-c - y_max*b)/a)

        p1 = np.array([x1, y_min]).astype('float32')
        p2 =  np.array([x2, y_max]).astype('float32')

        r1_cam = pixel2ray(p1, intrinsics['mtx'], intrinsics['dist'])
        r2_cam = pixel2ray(p2, intrinsics['mtx'], intrinsics['dist'])

        r1_world = np.matmul(R.T, np.squeeze(r1_cam).T)
        r2_world = np.matmul(R.T, np.squeeze(r2_cam).T)
        
        #cam_center_world = camera2world(np.array([0,0,0]), extrinsics['rmat_h'], extrinsics['tvec_h'])
        cam_center_world =  np.matmul(-R.T, Tr)
        
        print(dX, dY, "LSDAS")
        obj_points = np.array([[0, dX, dX, 0], [0, 0, dY, dY], [0, 0, 0, 0]]).T.reshape(-1, 1, 3)

        plane_n, plane_d = plane_from_points(obj_points[0,0], obj_points[1,0], obj_points[2,0], obj_points[3,0])

        #print(plane_n.shape, plane_d, cam_center_world.shape, r1_world.shape)
        intersection1 = intersection_point(plane_n, plane_d, cam_center_world.T[0], r1_world)
        intersection2 = intersection_point(plane_n, plane_d, cam_center_world.T[0], r2_world)

        intersection1_cam = world2camera(intersection1, R, Tr)
        intersection2_cam = world2camera(intersection2, R, Tr)

        #print(intersection1_cam.shape, intersection2_cam.shape)
        output_points1.append(intersection1_cam)
        output_points2.append(intersection2_cam)

        output[t] = {'pt1': intersection1_cam, 'pt2': intersection2_cam}

    
    output_points1 = np.array(output_points1)
    output_points2 = np.array(output_points2)

    #print(output_points, "MM")
    
    #obj_points = world2camera(obj_points[:,0], R, Tr)
    obj_points = np.dot(R, obj_points[:,0].T) + Tr
    print(obj_points.shape, "MM")
    #obj_points = np.array([[0, dX, dX, 0], [0, 0, dY, dY], [0, 0, 0, 0]]).T.reshape(-1, 1, 3)
    obj_points = obj_points.T
    #obj_points = obj_points[:,0,:]

    ax.plot(obj_points[:,0], obj_points[:,1], obj_points[:,2], color = 'red')
    #ax.plot(0,0,0, color = 'blue', marker = 'o', markersize = 10)
    
    ax.plot(cam_center_world[0], cam_center_world[1], cam_center_world[2], color = 'blue', marker = 'o', markersize = 10)
   

    temp = cam_center_world[:,0]  + r1_world*3156
    print(temp.shape, r1_world.shape, cam_center_world.shape)
    ax.plot(temp[0], temp[1], temp[2] , color = 'yellow', marker = 'o', markersize = 10)
    temp = cam_center_world[:,0] + r1_world*800

    ax.plot(temp[0], temp[1], temp[2] , color = 'yellow', marker = 'o', markersize = 10)

    ax.plot(output_points1[:,0], output_points1[:,1], output_points1[:,2], color = 'green', marker = 'o', markersize = 2)
    ax.plot(output_points2[:,0], output_points2[:,1], output_points2[:,2], color = 'blue', marker = 'o', markersize = 2)
    
    ax.set_box_aspect([1,1,1])

    #plt.show()

    return output



if __name__ == "__main__":

    data_dir = 'custom/box4'
    data, img_list = read_files(data_dir)
    intrinsics = np.load('custom/calib2/intrinsic_calib.npz')
    extrinsics = np.load(f'{data_dir}/extrinsic_calib.npz')

    fig = plt.figure("Projected camera view", figsize=(12,12))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    dX = 300.0 #calibration plane length in x direction
    dY = 255.0 #calibration plane length in y direction

    I_max = np.max(data, axis = 0)
    I_min = np.min(data, axis = 0)

    I_shadow = (I_max + I_min)/2

    I_delta = data - np.expand_dims(I_shadow, axis = 0)

    # x_min , x_max = 194, 332
    # y_min, y_max = 6, 54

    # 551,66,1352,427

    #x_min, y_min, x_max, y_max = 551,66,1352,427

    #x_min, y_min, x_max, y_max =  643,17,1399,371

    #x_min, y_min, x_max, y_max = 684,119,1398,427

    #x_min, y_min, x_max, y_max = 566,33,1208,220

    x_min, y_min, x_max, y_max =  743,102,1316,385

    prev_ymax = y_max

    output_vertical = get_shadow_line(x_min, x_max, y_min, y_max, I_delta, img_list, viz=False)
    output_points_vertical = get_plane_intersections(
        output_vertical, 
        intrinsics, 
        extrinsics['rmat_v'], 
        extrinsics['tvec_v'],
        y_min,
        y_max)
        
    # y_min, y_max = 221, 269

    # x_min, x_max = 232, 280

    #x_min, y_min, x_max, y_max =723,868,1340,1066

    #x_min, y_min, x_max, y_max = 704,883,1397,1079

    # Box 4
    x_min, y_min, x_max, y_max = 767,909,1375,1079

    #x_min, y_min, x_max, y_max = 952,1022,1501,1066

    # Box 9
    #x_min, y_min, x_max, y_max = 724,899,1304,1066

    output_horizontal = get_shadow_line(x_min, x_max, y_min, y_max, I_delta, img_list)

    #print(I_delta.shape)

    

    output_points_horizontal = get_plane_intersections(
        output_horizontal, 
        intrinsics, 
        extrinsics['rmat_h'], 
        extrinsics['tvec_h'],
        y_min,
        y_max)

    I_out = get_per_pixel_shadow(I_delta)

    # plt.show()

    # plt.imshow(I_out)
    # plt.show()

    fig = plt.figure("Projected camera view", figsize=(12,12))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')


    scan_3d = []
    color = []

    thresh = 0.0

    x_min, y_min, x_max, y_max = 775,402,1340,894
    #x_min, y_min, x_max, y_max = 900,560,1255,914

    #x_min, y_min, x_max, y_max = 701,222,1192,961

    #Box 9
    #x_min, y_min, x_max, y_max = 746,546,1296,895

    for i in tqdm(range(y_min, y_max)):
        for j in range(x_min, x_max):
            
            t = I_out[i][j]
            diff = I_max[i][j] - I_min[i][j]
            #print(diff)
            if abs(diff) > thresh:

                if t in output_points_horizontal.keys() and t in output_points_vertical.keys():
                    #print(i, j, t)
                    p1, p2 = output_points_horizontal[t]['pt1'], output_points_horizontal[t]['pt2']
                    p3, p4 = output_points_vertical[t]['pt1'], output_points_vertical[t]['pt2']

                    n,d = plane_from_points_four(p1, p2, p3, p4)
                    cam_center_world = np.array([0,0,0])

                    p = np.array([j, i]).astype('float32')
                    r_cam = pixel2ray(p, intrinsics['mtx'], intrinsics['dist'])

                    
                    intersection1 = intersection_point(n, d, cam_center_world, r_cam[0,0])

                    if True:
                        #print("HERE")
                        scan_3d.append(intersection1)
                        #print(intersection1.shape)
                        #r_world = np.matmul(R.T, np.squeeze(r1_cam).T)
                        

                        temp = np.vstack([p1, p2, p3, p4])
                        clr = data[0, i, j]
                        #print(clr)
                        color.append(clr)
                        #print(temp.shape, "GG")
                        #ax.plot(temp[:,0], temp[:,1], temp[:,2],  color = 'yellow', marker = 'o')

                        #temp = cam_center_world + 1901*r_cam[0,0]
                        #ax.plot(intersection1[0], intersection1[1], intersection1[2],  color = 'blue', marker = 'o')
                        #break
                else:
                    pass
                    #print(i,j,t)
            #break
        
    #ax.plot(cam_center_world[0], cam_center_world[1], cam_center_world[2], marker = 'o', color = 'green')
                
    scan_3d = np.array(scan_3d)
    color = np.array(color)

    print(color.shape, scan_3d.shape)
    import plotly.graph_objects as go

    print(scan_3d.shape)
    fig = go.Figure(data=[go.Scatter3d(z=scan_3d[:,2], x=scan_3d[:,0], y=scan_3d[:,1],  mode='markers', 
            marker=dict(
                size=2,
                color=color,  # set color to an array/list of desired values
                colorscale='gray',   # choose a colorscale
                ))])
    fig.update_layout(title='3D Surface Plot from NumPy Array',
                    scene=dict(
                        xaxis_title='X-axis',
                        yaxis_title='Y-axis',
                        zaxis_title='Z-axis'
                    ))
    fig.show()

    n = 3

    scan_3d = scan_3d[::n]
    ax.scatter(scan_3d[:,0], scan_3d[:,1], scan_3d[:,2],  marker = '.')
    
    set_axes_equal(ax)
    plt.show()
    
    

