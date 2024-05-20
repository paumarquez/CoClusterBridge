import numpy as np
import torch
import time

from control_cluster_bridge.controllers.rhc import RHController
from control_cluster_bridge.cluster_client.control_cluster_client import ControlClusterClient


class Controller(RHController):
    def _update_open_loop(self):
        # updates rhc controller
        # using the internal state
        pass

    def _update_closed_loop(self):
        # uses meas. from robot
        pass

    def _solve(self) -> bool:
        print("Solving", self.controller_index)
        
        if self.controller_index == 0:
            print("Sleeping", self.controller_index)
            time.sleep(5)
        else:
            time.sleep(1)
        print("Solved time", self.controller_index)
        q = self.robot_state.jnts_state.get(data_type="q", robot_idxs=[self.controller_index])
        print("q", self.controller_index)

        self.sol_q = q + 10
        self.sol_v = q + 11
        self.sol_eff = q + 12
        return True

    def _get_ndofs(self):
        return 7

    def _init_problem(self):
        print("Init problem")
        self._assign_controller_side_jnt_names(list(map(str, range(self._get_ndofs()))))
        pass

    def _reset(self):   
        print("resetting")
        self._init_problem()
        pass

    def _init_rhc_task_cmds(self):
        pass

    def _get_robot_jnt_names(self):
        pass

    def _get_contact_names(self):
        return list(map(str, range(self._get_ndofs())))

    def _get_cmd_jnt_q_from_sol(self) -> np.ndarray:
        return self.sol_q

    def _get_cmd_jnt_v_from_sol(self) -> np.ndarray:
        return self.sol_v

    def _get_cmd_jnt_eff_from_sol(self) -> np.ndarray:
        return self.sol_eff


N_NODES=12
DT=0.01
NAMESPACE="test"

class ClusterClient(ControlClusterClient):
    def _generate_controller(self,
                        idx: int):
        print("generating controller", idx)
        return Controller(self.cluster_size, DT, self._namespace)

if __name__ == "__main__":
    try:
        c = ClusterClient(NAMESPACE, N_NODES, isolated_cores_only=False)
        print("About to run")
        c.run()

    except Exception as e:
        print(e)
        c.terminate()
