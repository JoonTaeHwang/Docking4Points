
import time
import copy
import numpy as np 
import matplotlib.pyplot as plt
from more_itertools import nth
from ouster import client, pcap
from typing import Optional
from DKPoints import SETTINGS, LiDAR, DK
import DKPoints.ICP as ICP

class SetParameters:
    def __init__(self):
        self.settings: Optional[SETTINGS] = None
        self.LiDAR: Optional[LiDAR] = LiDAR
        self.DK: Optional[DK] = DK

class Builder:
    def __init__(self,config = SetParameters()):
        self.config = config
    
    def settings(self,settings):
        self.config.settings = settings
        return self
    
    def build(self):
        GetDKpoints(self.config)
        return self

class LiDARHandler:
    def __init__(
        self,
        Config = SetParameters()
    ):
        self.config = Config
        self.SETLiDAR()
        
    def SETLiDAR(self):
        try:
            with open(self.config.settings.metadata_path, 'r') as f:
                self.config.LiDAR.metadata = client.SensorInfo(f.read())
                self.config.LiDAR.source = pcap.Pcap(self.config.settings.pcap_path, self.config.LiDAR.metadata)
        except Exception as e:
            print("Error occurred in SETLiDAR:", e)
            time.sleep(0.1)

    def get_lidardata(self):
        try:
            scan = nth(client.Scans(self.config.LiDAR.source), 0)
            xyzlut = client.XYZLut(self.config.LiDAR.metadata)
            xyz = xyzlut(scan.field(client.ChanField.RANGE))
            key = scan.field(client.ChanField.REFLECTIVITY)
            reflectivity = key.flatten()
            COORD_TRANS_MTX=np.array([
                                    [0, 1, 0],
                                    [-1, 0, 0],
                                    [0, 0, 1]
                                    ])
            pcd = np.asarray([c.flatten() for c in np.dsplit(xyz, 3)]).T @ COORD_TRANS_MTX
            # 자선 중심기준 80m로 제한
            valid=np.where((pcd[:,0]>-40)&(pcd[:,0]<40)&(pcd[:,1]>-40)&(pcd[:,1]<40)&(pcd[:,2]>0)&(reflectivity>12000))
            point_valid=np.round(pcd[valid][:,0:2]/self.config.settings.voxel_size)*self.config.settings.voxel_size
            temp_u_pcd,indices =np.unique(point_valid, return_counts = True, axis=0)
            filt=np.where(indices>1)
            self.config.LiDAR.voxel2d = temp_u_pcd[filt]
        except Exception:
            ("no data")
            self.config.LiDAR.running = False
        
class GetDKpoints:
    def __init__(
        self,
        Config = SetParameters()
    ):
        self.LiDAR = LiDARHandler(Config)
        self.config = Config
        self.config.LiDAR.voxel2d = np.empty(0)
        self.config.LiDAR.voxel2d_prev = np.empty(0)
        self.SetDKposition()
        
    def SetDKposition(self):
        self.LiDAR.get_lidardata()
        fig= plt.figure(1,figsize=(8,8))
        ax = fig.add_subplot()
        ax.axis([-40,40,-40,40])    
        plt.scatter(self.config.LiDAR.voxel2d[:, 1], self.config.LiDAR.voxel2d[:, 0],c='green', marker='s', picker = True ,s=int(self.config.settings.voxel_size*20),alpha=0.30) 
        plt.grid(True)
        plt.grid(alpha = 0.3)
        while True:
            clicked_points = plt.ginput(n=1, timeout=0)
            if len(clicked_points) > 0:
                clicked_x, clicked_y = clicked_points[0]  
                ax.scatter(clicked_x, clicked_y, c='red', marker='x')
                self.config.DK.center_point = np.array([clicked_y,clicked_x])
                self.config.DK.dkpoints =np.array(
                    [[self.config.settings.boatwidth/2,self.config.settings.boatlenth/2,0],
                     [-self.config.settings.boatwidth/2,self.config.settings.boatlenth/2,0],
                     [-self.config.settings.boatwidth/2,-self.config.settings.boatlenth/2,0],
                     [self.config.settings.boatwidth/2,-self.config.settings.boatlenth/2,0]]) + np.array([self.config.DK.center_point[0], self.config.DK.center_point[1],0])
                self.GetDKposition()
                break
            # else:
            #     time.sleep(0.01)
                
    def GetDKposition(self):
        while self.config.LiDAR.running:
            self.LiDAR.get_lidardata()
            # ICP for DK area
            if len(self.config.LiDAR.voxel2d_prev)>0:
                T, _, _, _, _ = ICP.icp(self.config.LiDAR.voxel2d_prev, self.config.LiDAR.voxel2d)# self.config.LiDAR.voxel2d_icp)
                m = box.shape[1]
                temp_box = np.ones((3,box.shape[0]))
                temp_box[:m,:] = np.copy(box.T)
                temp_box = np.dot(T, temp_box)
                box= temp_box.T  
                self.config.DK.center_point=np.sum(box,axis=0)/len(box)
            else:
                box=np.array([[self.config.settings.boatwidth/2,self.config.settings.boatlenth/2,0],[-self.config.settings.boatwidth/2,self.config.settings.boatlenth/2,0],[-self.config.settings.boatwidth/2,-self.config.settings.boatlenth/2,0],[self.config.settings.boatwidth/2,-self.config.settings.boatlenth/2,0]])+np.array([self.config.DK.center_point[0], self.config.DK.center_point[1],0])                                 
            self.config.LiDAR.voxel2d_prev=copy.deepcopy(self.config.LiDAR.voxel2d)
            
            # 주차영역 중심으로 10m 구간으로 제한
            valid=np.where((self.config.LiDAR.voxel2d[:,0]>-15+self.config.DK.center_point[0])
                        &(self.config.LiDAR.voxel2d[:,0]<15+self.config.DK.center_point[0])
                        &(self.config.LiDAR.voxel2d[:,1]>-15+self.config.DK.center_point[1])
                        &(self.config.LiDAR.voxel2d[:,1]<15+self.config.DK.center_point[1]))
            voxel2d_dk_area=self.config.LiDAR.voxel2d[valid]
            
            # 주차영역을 선석 안쪽으로 제한하기 위해 선석 안쪽으로 이동
            h_vec=(box[3,:]-box[0,:])/np.linalg.norm(box[3,:]-box[0,:])
            # w_vec=(box[1,:]-box[0,:])/np.linalg.norm(box[1,:]-box[0,:])
            ctr_vec=box[0,:] /np.linalg.norm(box[0,:])
            hc_dot=np.dot(h_vec,ctr_vec)
            self.config.DK.dkpoints,dk_clear=self.GetDKpoints(voxel2d_dk_area, self.config.DK.dkpoints, h_vec*np.sign(hc_dot))
            self.visualize(dk_clear)
            
    def GetDKpoints(self, voxel2d_dk_area, box_o, h_vec):
        box=box_o+h_vec
        cnt= self.define_inside_box_2D(box,voxel2d_dk_area)
        pcd_in_box=voxel2d_dk_area[np.where(cnt==4)]
        dk_clear=False      
        # start_time = time.time()
        for k in range(90):
            count=0
            if(k%2==0):
                rot_ang=k*1
            else:
                rot_ang=-k*1
            self.config.DK.center_point=np.sum(box,axis=0)/len(box)
            box=self.rotate_point(box,self.config.DK.center_point,np.array([0,0,rot_ang*np.pi/180]))
            box_b=copy.deepcopy(box)
            while len(pcd_in_box)>0:
                count+=1
                dis_index=[]
                dis=[]
                for i in range(len(pcd_in_box)):         
                    temp_dis=[]
                    
                    #box의 한 변과 box안의 pcd 사이의 거리
                    for j in range(len(box)):
                        vertex_x1 = box[j, 0]
                        vertex_y1 = box[j, 1]
                        vertex_x2 = box[(j + 1) % len(box), 0]
                        vertex_y2 = box[(j + 1) % len(box), 1]
                        temp_dis.append(abs((vertex_x2-vertex_x1)*(vertex_y1-pcd_in_box[i,1])
                                         -(vertex_x1-pcd_in_box[i,0])*(vertex_y2-vertex_y1))
                                        /np.sqrt((vertex_x2-vertex_x1)**2+(vertex_y2-vertex_y1)**2)
                                        )               
                    dis_index.append(temp_dis.index(min(temp_dis)))       
                    dis.append(min(temp_dis))
                sort_index = np.argsort(dis)[::-1]    
                if len(dis)>0:
                    h_vec=(box[3,:]-box[0,:])/np.linalg.norm(box[3,:]-box[0,:])
                    w_vec=(box[1,:]-box[0,:])/np.linalg.norm(box[1,:]-box[0,:])
                    
                    if dis_index[sort_index[0]]==0:
                        box=box+h_vec*(dis[sort_index[0]])*1.01
                    elif dis_index[sort_index[0]]==1:
                        box=box-w_vec*(dis[sort_index[0]])*1.01
                    elif dis_index[sort_index[0]]==2:
                        box=box-h_vec*(dis[sort_index[0]])*1.01
                    elif dis_index[sort_index[0]]==3:
                        box=box+w_vec*(dis[sort_index[0]])*1.01  

                    check_cp_in_DKarea = self.define_inside_box_2D(box,np.asarray([self.config.DK.center_point]))
                    if check_cp_in_DKarea<4:
                        box=copy.deepcopy(box_b)
                        break
                    cnt = self.define_inside_box_2D(box,voxel2d_dk_area)                
                    pcd_in_box=voxel2d_dk_area[np.where(cnt==4)]   
                    self.config.DK.center_point=np.sum(box,axis=0)/len(box)
                    #=============================
                    
                if count==3:
                    break
            if len(pcd_in_box)==0:
                dk_clear=True
                break
            else:
                box=box_o
                
        return box,dk_clear
    
    def define_inside_box_2D(self, box, pcd):
        cnt=np.sign((box[1,0]-box[0,0])*(pcd[:,1]-box[0,1])-(box[1,1]-box[0,1])*(pcd[:,0]-box[0,0]))
        cnt+=np.sign((box[2,0]-box[1,0])*(pcd[:,1]-box[1,1])-(box[2,1]-box[1,1])*(pcd[:,0]-box[1,0]))
        cnt+=np.sign((box[3,0]-box[2,0])*(pcd[:,1]-box[2,1])-(box[3,1]-box[2,1])*(pcd[:,0]-box[2,0]))
        cnt+=np.sign((box[0,0]-box[3,0])*(pcd[:,1]-box[3,1])-(box[0,1]-box[3,1])*(pcd[:,0]-box[3,0]))   
        return np.abs(cnt)
    
    def rotate_point(self,point,ctr,rotate_angle):
        Rx=rotate_angle[0]
        Ry=rotate_angle[1]
        Rz=rotate_angle[2]
        #====== Rotational  matrix ======
        rot_x = np.array([[1,       0,        0],                 
                [0,    np.cos(Rx),  np.sin(Rx)],   
            [0,  -np.sin(Rx),  np.cos(Rx)]])
            
        rot_y = np.array([[np.cos(Ry),  0,   -np.sin(Ry)],            
                [0,        1,       0],          
                [np.sin(Ry),  0,    np.cos(Ry)]])
            
        rot_z = np.array([[ np.cos(Rz),  np.sin(Rz),   0],   
                [-np.sin(Rz),  np.cos(Rz),   0],   
                [0,        0,           1]])     
        #===============================
        if(Rz != 0):
            point  =  ctr+(point-ctr)@rot_z
        if(Ry != 0):
            point  =  ctr+(point-ctr)@rot_y
        if(Rx != 0):
            point  =  ctr+(point-ctr)@rot_x
        return point
    
    def visualize(self,dk_clear):
        # Visualize 4pts
        fig= plt.figure(1,figsize=(10,10))
        ax = fig.add_subplot()
        ax.axis([-40,40,-40,40])    
        ax.scatter(self.config.DK.center_point[1], self.config.DK.center_point[0], c='red', marker='x')
        plt.scatter(self.config.LiDAR.voxel2d[:, 1], self.config.LiDAR.voxel2d[:, 0],c='green', marker='s', picker = True ,s=int(self.config.settings.voxel_size*20),alpha=0.30) 
        if dk_clear:
            plt.scatter(self.config.DK.dkpoints[:, 1], self.config.DK.dkpoints[:, 0],c='red', marker='^', picker = True ,s=25)     
        else:
            plt.scatter(self.config.DK.dkpoints[:, 1], self.config.DK.dkpoints[:, 0],c='yellow', marker='^', picker = True ,s=25) 
        plt.grid(True)
        plt.grid(alpha = 0.3)
        plt.pause(0.1)
        fig.clear()
        