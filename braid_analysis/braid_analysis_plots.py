import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

def plot_3d_trajectory(df_3d, 
                       obj_ids=None,
                       frame_range=None,
                       mode='time', 
                       color=None,
                       xlim=[-0.5, 0.5], ylim=[-0.25, 0.25], zlim=[0, 0.5]):
    '''
    either obj_ids or frame_range must not be none
    
    obj_ids can be a list, in which case, multiple trajecs will be plotted
    frame_range should be a list with first and last frame of interest
    
    mode: 
       time = plot x,y,z,speed vs. time
       3d   = plot x vs y, x vs z
       
    '''
    if obj_ids is None:
        assert frame_range != None
        df_3d_q = df_3d.query('frame > ' + str(frame_range[0]) + ' and frame < ' + str(frame_range[-1]) )
        obj_ids = df_3d_q.obj_id.unique()
    
    try: 
        _ = obj_ids[0]
    except:
        obj_ids = [obj_ids]

    fig = plt.figure()
    
    # XY XZ planes
    if mode == '3d':
        ax1 = fig.add_subplot(121)
        ax2 = fig.add_subplot(122)

        for oid in obj_ids:
            trajec = df_3d[df_3d.obj_id==oid]
            ax1.plot(trajec.x, trajec.y)
            ax2.plot(trajec.x, trajec.z)

        for i, ax in enumerate([ax1, ax2]):
            ax.set_aspect('equal')
            ax.set_xlabel('x')
            if i == 0:
                ax.set_ylabel('y')
            else:
                ax.set_ylabel('z')
    
    # X,Y,Z,speed vs time
    elif mode == 'time':
        ax1 = fig.add_subplot(411)
        ax1.set_ylim(*xlim)
        ax1.set_xlim(*frame_range)
        ax1.set_ylabel('x pos')
        ax2 = fig.add_subplot(412)
        ax2.set_ylim(*ylim)
        ax2.set_xlim(*frame_range)
        ax2.set_ylabel('y pos')
        ax3 = fig.add_subplot(413)
        ax3.set_ylim(*zlim)
        ax3.set_xlim(*frame_range)
        ax3.set_ylabel('z pos')
        ax4 = fig.add_subplot(414)
        ax4.set_ylim(0, 1.5)
        ax4.set_xlim(*frame_range)
        ax4.set_ylabel('speed')
        ax4.set_xlabel('Frames')
        
        for oid in obj_ids:
            trajec = df_3d[df_3d.obj_id==oid]
            line, = ax1.plot(trajec.frame, trajec.x)
            ax1.fill_between(trajec.frame, trajec.x+trajec.P00**0.5, trajec.x-trajec.P00**0.5, 
                             color=line.get_color(), alpha=0.3)
            
            ax2.plot(trajec.frame, trajec.y)
            ax2.fill_between(trajec.frame, trajec.y+trajec.P11**0.5, trajec.y-trajec.P11**0.5, 
                             color=line.get_color(), alpha=0.3)
            
            ax3.plot(trajec.frame, trajec.z)
            ax3.fill_between(trajec.frame, trajec.z+trajec.P22**0.5, trajec.z-trajec.P22**0.5, 
                             color=line.get_color(), alpha=0.3)
            
            speed = np.sqrt(trajec.xvel**2 + trajec.yvel**2 + trajec.zvel**2)
            ax4.plot(trajec.frame, speed)
            
def plot_2d_data(df_2d, df_3d, frame_range=None):
    first_frame, last_frame = frame_range
    
    
    if first_frame is None:
        first_frame = df_2d.frame.values[0]
    if last_frame is None:
        last_frame = df_2d.frame.values[-1]
        
    frames = np.arange(first_frame, last_frame+1)
    df_3d_traj = df_3d[df_3d['frame'].isin(frames)]
    
    color = np.array([0.5]*len(frames))
    color[df_3d_traj.frame.values - frames[0]] = 1

    # get 2d data 
    df_2d_traj = df_2d[df_2d['frame'].isin(frames)]

    # Find all cameras
    camns = df_2d_traj.camn.unique()
    camns = np.sort(camns)

    # plot the data
    fig = plt.figure(figsize=(10,10))
    xticks = [first_frame, last_frame]
    for camn in camns:
        ax = fig.add_subplot(len(camns), 2, 2*camn+1)
        df_2d_traj_camn_frames = df_2d_traj[df_2d_traj['camn']==camn].frame.values
        ax.scatter( df_2d_traj[df_2d_traj['camn']==camn].frame, 
                    df_2d_traj[df_2d_traj['camn']==camn].x,
                    c=color[df_2d_traj_camn_frames-first_frame], vmin=0.5, vmax=1)
        ax.set_ylabel('Cam: ' + str(camn))

        # plot labels
        #ax.set_xlim(first_frame, last_frame)
        ax.set_xticks(xticks)
        ax.set_ylim(0, 800)
        ax.set_yticks([0, 800])
        if camn==0:
            ax.set_title('x pixel')
        if camn != camns[-1]:
            ax.set_xticklabels([])
        else:
            ax.set_xlabel('Frames')

        ax = fig.add_subplot(len(camns), 2, 2*camn+2)
        ax.scatter(df_2d_traj[df_2d_traj['camn']==camn].frame, 
                   df_2d_traj[df_2d_traj['camn']==camn].y,
                   c=color[df_2d_traj_camn_frames-first_frame], vmin=0.5, vmax=1)

        # plot labels
        #ax.set_xlim(first_frame, last_frame)
        ax.set_xticks(xticks)
        ax.set_ylim(0, 800)
        ax.set_yticks([0, 800])
        if camn==0:
            ax.set_title('y pixel')
        if camn != camns[-1]:
            ax.set_xticklabels([])
        else:
            ax.set_xlabel('Frames')

def plot_2d_datums_per_camera(df_2d, df_3d, frame_range=None):
    first_frame, last_frame = frame_range
    
    
    if first_frame is None:
        first_frame = df_2d.frame.values[0]
    if last_frame is None:
        last_frame = df_2d.frame.values[-1]
        
    frames = np.arange(first_frame, last_frame+1)
    df_3d_traj = df_3d[df_3d['frame'].isin(frames)]
    
    color = np.array([0.5]*len(frames))
    color[df_3d_traj.frame.values - frames[0]] = 1

    # get 2d data 
    df_2d_traj = df_2d[df_2d['frame'].isin(frames)]

    # Find all cameras
    camns = df_2d_traj.camn.unique()
    camns = np.sort(camns)

    # plot the data
    fig = plt.figure(figsize=(15,10))
    xticks = [first_frame, last_frame]
    datums_per_camera = []
    for camn in camns:
        ax = fig.add_subplot(len(camns)+2, 1, camn+1)
        
        df_2d_traj_camn_frames = df_2d_traj[df_2d_traj['camn']==camn].frame.unique()
        df_2d_traj['x_not_nan'] = ~np.isnan(df_2d_traj.x)
        n_datums = df_2d_traj[df_2d_traj['camn']==camn].groupby('frame').sum().x_not_nan
        
        ax.plot( df_2d_traj_camn_frames, 
                    n_datums, c='black')
        ax.set_ylabel('Cam: ' + str(camn))

        # plot labels
        #ax.set_xlim(first_frame, last_frame)
        ax.set_xticks(xticks)
        ax.set_xlim(*frame_range)
        ax.set_ylim(0, 3)
        ax.set_yticks([0, 1, 2, 3])
        if camn==0:
            ax.set_title('number of 2d datums')
        if camn != camns[-1]:
            ax.set_xticklabels([])
        else:
            ax.set_xticklabels([])
            #ax.set_xlabel('Frames')

        datums_per_camera.append(n_datums)
        
    # how many cameras total?
    ax_all = fig.add_subplot(len(camns)+2, 1, len(camns)+1)
    ax_all.plot(df_2d_traj_camn_frames, np.sum(datums_per_camera, axis=0))
    ax_all.set_ylabel('N cams')
    ax_all.set_ylim(0,9)
    ax_all.set_yticks([0,3,6,9])
    ax_all.set_xlim(*frame_range)
            
    # now plot x position of all the objects to see ghosts
    df_3d_q = df_3d.query('frame > ' + str(frame_range[0]) + ' and frame < ' + str(frame_range[-1]) )
    obj_ids = df_3d_q.obj_id.unique()
    
    ax_x = fig.add_subplot(len(camns)+2, 1, len(camns)+2)
    for oid in obj_ids:
        trajec = df_3d[df_3d.obj_id==oid]
        ax_x.plot(trajec.frame, trajec.x)
        ax_x.set_xlim(*frame_range)
    
    ax_x.set_xticks(xticks)
    ax_x.set_ylim(-0.3)
    ax_x.set_yticks([0.3])
    ax_x.set_xlabel('Frames')
    ax_x.set_ylabel('x pos')