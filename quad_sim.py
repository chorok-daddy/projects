import quadcopter,gui,controller
import signal
import sys
import argparse
from PSO.Drone import *

# Constants
TIME_SCALING = 0.0 # Any positive number(Smaller is faster). 1.0->Real Time, 0.0->Run as fast as possible
QUAD_DYNAMICS_UPDATE = 0.01 # seconds
CONTROLLER_DYNAMICS_UPDATE = 0.015 # seconds
run = True

def Multi_Point2Point():
    # PSO iterations s
    num_drones = 15
    num_iters = 30
    func_num = 1
    swarm = Swarm(func_num, num_drones)
    # Set Drones
    Drones = {}
    Trajs = []
    for drone in swarm.drones:
        name = 'd'+str(swarm.drones.index(drone)+1)
        dic = {}
        dic['position'] = (drone.pos/10.0).tolist()
        dic['orientation'] = [0,0,0.9]
        dic['L'] = 0.3
        dic['r'] = 0.1
        dic['prop_size'] = [10, 4.5]
        dic['weight'] = 1.2    
        Drones[name] = dic

    # Set Controller Parameters
    CONTROLLER_PARAMETERS = {'Motor_limits':[4000,9000],
                        'Tilt_limits':[-10,10],
                        'Yaw_Control_Limits':[-900,900],
                        'Z_XY_offset':500,
                        'Linear_PID':{'P':[300,300,7000],'I':[0.04,0.04,4.5],'D':[450,450,5000]},
                        'Linear_To_Angular_Scaler':[1,1,0],
                        'Yaw_Rate_Scaler':0.18,
                        'Angular_PID':{'P':[22000,22000,1500],'I':[0,0,1.2],'D':[12000,12000,0]},
                        }

    # Catch Ctrl+C to stop threads
    signal.signal(signal.SIGINT, signal_handler)

    # Make objects for quadcopter, gui and controllers
    drones = quadcopter.Quadcopter(quads=Drones)
    gui_object = gui.GUI(func_num = func_num, quads=Drones)
    drones.start_thread(dt=QUAD_DYNAMICS_UPDATE,time_scaling=TIME_SCALING)
    ctrls = {}
    for i in range(0,num_drones):
        name = 'd'+str(i+1)
        ctrls[name] = controller.Controller_PID_Point2Point(drones.get_state,drones.get_time,drones.set_motor_speeds,params=CONTROLLER_PARAMETERS,quad_identifier=name)
        ctrls[name].start_thread(update_rate=CONTROLLER_DYNAMICS_UPDATE,time_scaling=TIME_SCALING)
        
    # Update the GUI while switching between destination poitions
    ground_truth = (0, 0, 10)
    for iter in range(0,num_iters):
        swarm.update_drones()
        cur_best = (swarm.gb_pos[0],swarm.gb_pos[1],swarm.gb_pos[2])
        if np.linalg.norm(np.array(ground_truth)-np.array(cur_best))<0.3:
            break
        text1 = '- Iteration %d -' % (iter+1)
        text2 = 'Iteration %d: ' % (iter+1) + 'Best at (%.3f, %.3f, %.3f)' % cur_best 
        gui_object.update_text(text1,0.42, 0.85, color='green')
        gui_object.update_gb_marker(swarm.gb_pos[0],swarm.gb_pos[1],swarm.gb_pos[2])
        print(text2) 
        for j in range(0,num_drones):
            name = 'd'+str(j+1)
            ctrls[name].update_target((swarm.drones[j].pos/10).tolist())
        for i in range(40):
            idx = 0
            for key in Drones:
                gui_object.quads[key]['position'] = drones.get_position(key)
                swarm.drones[idx].update_pb_online(drones.get_position(key))
                gui_object.quads[key]['orientation'] = drones.get_orientation(key)
                idx+=1
            gui_object.update()  
    drones.stop_thread()
    for i in range(0,num_drones):
        name = 'd'+str(i+1)
        ctrls[name].stop_thread()
    swarm.update_drones()
    text = 'Best found at (%.2f, %.2f, %.2f)' % (swarm.gb_pos[0],swarm.gb_pos[1],swarm.gb_pos[2])
    gui_object.final_report(text, 0.3, 0.75, color='black') 

def parse_args():
    parser = argparse.ArgumentParser(description="Quadcopter Simulator")
    parser.add_argument("--sim", help='single_p2p, multi_p2p or single_velocity', default='single_p2p')
    parser.add_argument("--time_scale", type=float, default=-1.0, help='Time scaling factor. 0.0:fastest,1.0:realtime,>1:slow, ex: --time_scale 0.1')
    parser.add_argument("--quad_update_time", type=float, default=0.0, help='delta time for quadcopter dynamics update(seconds), ex: --quad_update_time 0.002')
    parser.add_argument("--controller_update_time", type=float, default=0.0, help='delta time for controller update(seconds), ex: --controller_update_time 0.005')
    return parser.parse_args()

def signal_handler(signal, frame):
    global run
    run = False
    print('Stopping')
    sys.exit(0)

if __name__ == "__main__":
    args = parse_args()
    if args.time_scale>=0: TIME_SCALING = args.time_scale
    if args.quad_update_time>0: QUAD_DYNAMICS_UPDATE = args.quad_update_time
    if args.controller_update_time>0: CONTROLLER_DYNAMICS_UPDATE = args.controller_update_time
    Multi_Point2Point()

