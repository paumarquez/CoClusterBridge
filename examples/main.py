import torch
import numpy as np

from control_cluster_bridge.cluster_server.control_cluster_server import ControlClusterServer

N_NODES=12
DT=0.01
NAMESPACE="test"

import time

if __name__ == "__main__":
    try:
        joint_names=list(map(str, range(7)))
        c_server = ControlClusterServer(NAMESPACE, N_NODES, DT, DT, jnt_names=joint_names, n_contact_sensors=7, contact_linknames=joint_names, force_reconnection=True)
        c_server._remote_triggerer_ack_timeout = 10000  # [ns]
        c_server.run()
        idx = 0
        while True:
            c_server.pre_trigger()
            rhc_state = c_server.get_state()
            print(rhc_state.jnts_state.get(data_type="q", robot_idxs=[0]))
            data = np.zeros((N_NODES, 7))
            data[0, 0] = idx
            data[1,0] = idx+1
            rhc_state.jnts_state.set(data=torch.tensor(data, dtype=torch.float32), data_type="q", robot_idxs=list(range(N_NODES)), gpu=False)
            # rhc_state.jnts_state.set(data=torch.tensor([[idx, 0, 0, 0, 0, 0, 0]], dtype=torch.float32), data_type="q", robot_idxs=[1], gpu=False)
            c_server.trigger_solution()
            print("waiting")
            c_server.wait_for_solution()
            v = c_server._rhc_cmds.jnts_state.get(data_type="v", robot_idxs=list(range(N_NODES)))
            print("solution done", v)
            time.sleep(1)
            idx += 2

    except Exception as e:
        print(e)
        c_server.close()