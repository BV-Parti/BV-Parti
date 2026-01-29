#!/usr/bin/python
import os
import time
import argparse
import subprocess

class Task():
    # sp: solving process
    # pp: partitioning process
    def __init__(self, id, parent, make_time):
        self.sp = None
        self.pp = None
        self.id = id
        self.parent = parent
        self.time_infos = {'make': make_time}
        
        # generating
        # waiting
        # simplifying simplified
        # solving
        # sat unsat unknown
        # generating  | -> waiting     BY (partition done)
        # waiting     | -> simplifying BY (simplify task)
        # simplifying | -> simplified  BY (simplification done)
        # simplified  | -> solving     BY (run task)
        # solving     | -> sat, unsat  BY (solver)
        # 
        # generating  |
        # waiting     |
        # simplifying |
        # simplified  |
        # solving     |
        #               -> unsat       BY (ancester, children, partitioner)
        #               -> unknown     BY (children)
        self.state = 'generating'
        # not-ready unpartitioned partitioning partitioned solved
        self.pstate = 'not-ready'
        self.reason = -3
        self.subtasks = []
        
    def __str__(self) -> str:
        pid = -1
        if (self.parent != None):
            pid = self.parent.id
        ret = f'id: {self.id}'
        ret += f', parent: {pid}'
        ret += f', state: {self.state}'
        if self.reason != -3:
            ret += f', reason: {self.reason}'
        if len(self.subtasks) > 0:
            stid = [self.subtasks[0].id, self.subtasks[1].id]
            ret += f', subtasks: {stid}'
        ret += f'\ntime-infos: {self.time_infos}\n'
        return ret

class BV_Parti():
    
    def get_current_time(self):
        return time.time() - self.start_time
    
    def init_params(self):
        arg_parser = argparse.ArgumentParser()
        arg_parser.add_argument('--file', type=str, required=True,
                                help='input instance file path')
        arg_parser.add_argument('--output-dir', type=str, required=True,
                                help='output dir path')
        arg_parser.add_argument('--partitioner', type=str, required=True,
                                help='partitioner path')
        arg_parser.add_argument('--solver', type=str, required=True,
                                help='solver path')
        arg_parser.add_argument('--max-running-tasks', type=int, default=32, 
                                help='maximum number of tasks running simultaneously')
        arg_parser.add_argument('--time-limit', type=int, default=0, 
                                help='time limit, 0 means no limit')
        args = arg_parser.parse_args()
        self.input_file_path: str = args.file
        self.output_dir_path: str = args.output_dir
        self.partitioner_path: str = args.partitioner
        self.solver_path: str = args.solver
        self.max_running_tasks: int = args.max_running_tasks
        self.time_limit: int = args.time_limit
        
        if not os.path.exists(self.input_file_path):
            print('file-not-found')
            assert(False)
        
        self.instance_name: str = self.input_file_path[ \
            self.input_file_path.rfind('/') + 1: self.input_file_path.find('.smt2')]
    
    def init_logging(self):
        log_file_path = f'{self.output_dir_path}/log'
        self.log_file = open(log_file_path, 'w')
        
        self.start_time = time.time()
        self.write_line_to_log(f'start-time {self.start_time}')
    
    def init(self):
        self.init_params()
        self.next_task_id = 0
        
        # self.generating_tasks = []
        # self.waiting_tasks = []
        # self.simplifying_tasks = []
        # self.simplified_tasks = []
        # self.solving_tasks = []
        # self.ended_tasks = []
        # self.terminated_tasks = []
        # self.unpartitioned_tasks = []
        # self.partitioning_tasks = []
        # self.partitioned_tasks = []
        
        self.state_tasks_dict = {
            'generating': [],
            'generated': [],
            'simplifying': [],
            'simplified': [],
            'solving': [],
            'ended': [],
        }
        
        self.pstate_tasks_dict = {
            'not-ready': [],
            'unpartitioned': [],
            'partitioning': [],
            'partitioned': [],
            'solved': [],
        }
        
        self.tasks = []
        self.result = 'undefined'
        self.reason = -3
        self.done = False
        
        self.max_unended_tasks = self.max_running_tasks + self.max_running_tasks // 3 + 1
        
        self.base_run_cnt = 0
        
        self.solve_ori_flag = True

        os.system(f'mkdir -p {self.output_dir_path}')
        os.system(f'mkdir -p {self.output_dir_path}/tasks')
        
        self.init_logging()
        
    
    def write_line_to_log(self, data: str):
        curr_time: int = int(time.time() - self.start_time)
        line: str = f'{curr_time} {data}\n'
        self.log_file.write(line)
        self.log_file.flush()
        
    def clean_up(self):
        # # if os.path.exists(f'{self.output_dir_path}'):
        # #     os.system(f'rm -r {self.output_dir_path}')
        # if os.path.exists(f'{self.output_dir_path}/tasks'):
        #     os.system(f'rm -r {self.output_dir_path}/tasks')
        pass
    
    def make_task(self, pid):
        parent: Task = None
        if pid != -1:
            parent = self.tasks[pid]
        t = Task(self.next_task_id, parent, self.get_current_time())
        self.next_task_id += 1
        if parent != None:
            parent.subtasks.append(t)
        self.tasks.append(t)
    
    def check_process(self, p: subprocess.Popen):
        rc = p.poll()
        if rc == None:
            return ['running']
        out_data, err_data = p.communicate()
        return ['done', rc, out_data, err_data]
    
    def check_simplifying_process(self, id, p: subprocess.Popen):
        res = self.check_process(p)
        if res[0] == 'running':
            return 'simplifying'
        rc, out_data, err_data = res[1], res[2], res[3]
        ret = 'simplified'
        lines = out_data.split('\n')
        self.write_line_to_log(f'simplifying-result id {id}')
        for line in lines:
            self.write_line_to_log(f'simplifying-result {line}')
        if False:
            print(rc)
            print(f'out_data: {out_data}, err_data: {err_data}')
        if rc != 0:
            ret = 'unknown'
            # raise AssertionError()
        self.write_line_to_log(f'task-simplified {id} {ret}')
        return ret
    
    def check_solving_process(self, id, p: subprocess.Popen):
        res = self.check_process(p)
        if res[0] == 'running':
            return 'solving'
        rc, out_data, err_data = res[1], res[2], res[3]
        ret: str = out_data.strip('\n').strip(' ')
        ret = 'unknown'
        lines = out_data.split('\n')
        self.write_line_to_log(f'solving-result id {id}')
        for line in lines:
            words = line.split(' ')
            if len(words) <= 0:
                continue
            if ret == 'unknown' and words[0] != 'c':
                ret = line
            self.write_line_to_log(f'solving-result {line}')
        if False:
            print(rc)
            print(f'out_data: {out_data}, err_data: {err_data}')
        self.write_line_to_log(f'return-code {rc}')
        # if rc != 0:
        #     ret = 'non-zero-return'
        #     raise AssertionError()
        self.write_line_to_log(f'task-solved {id} {ret}')
        return ret
    
    def check_partitioning_process(self, id, p: subprocess.Popen):
        res = self.check_process(p)
        if res[0] == 'running':
            return 'partitioning'
        rc, out_data, err_data = res[1], res[2], res[3]
        ret = 'partitioned'
        lines = out_data.split('\n')
        self.write_line_to_log(f'partitioning-result id {id}')
        for line in lines:
            self.write_line_to_log(f'partitioning-result {line}')
        if False:
            print(rc)
            print(f'out_data: {out_data}, err_data: {err_data}')
        if rc != 0:
            # ret = 'non-zero-return'
            if rc == 233:
                ret = 'solved'
            else:
                ret = 'unknown'
                # raise AssertionError()
        self.write_line_to_log(f'task-partitioned {id} {ret}')
        return ret
        
    def propagate_unsat(self, t: Task, reason):
        assert(t.state != 'unsat')
        self.update_task_state(t, 'unsat')
        t.reason = reason
    
    def push_up(self, t: Task, reason):
        # only 'unsat' 'unknown' need push up
        if t.state == 'unsat':
            return
        if len(t.subtasks) == 2 and \
           t.subtasks[0].state == 'unsat' and \
           t.subtasks[1].state == 'unsat':
            self.propagate_unsat(t, reason)
            self.write_line_to_log(f'unsat-by-children {t.id} {t.subtasks[0].id} {t.subtasks[1].id}')
            if t.parent != None:
                self.push_up(t.parent, reason)
    
    def push_down(self, t: Task, reason):
        # only 'unsat' need push up
        if t.state == 'unsat':
            return
        self.propagate_unsat(t, reason)
        self.write_line_to_log(f'unsat-by-ancestor {t.id} {reason}')
        for st in t.subtasks:
            self.push_down(st, reason)

    def need_terminate(self, t: Task):
        if t.id <= 0:
            return False
        num_st = len(t.subtasks)
        st_end = 0
        if num_st > 0 and t.subtasks[0].state in ['solving', 'unsat', 'terminated']:
            st_end += 1
        if num_st > 1 and t.subtasks[1].state in ['solving', 'unsat', 'terminated']:
            st_end += 1
        
        if st_end == 0:
            return False
        if st_end == 1 and self.get_current_time() - t.time_infos['solving'] < 200.0:
            return False
        if st_end == 2 and self.get_current_time() - t.time_infos['solving'] < 100.0:
            return False
        return True
    
    def update_task_state(self, t: Task, new_state: str):
        self.write_line_to_log(f'update-state {t.id} {new_state}')
        t.state = new_state
        if new_state == 'unsat':
            if t.pp != None:
                t.pp.terminate()
            self.update_task_pstate(t, 'solved')
        if new_state in ['sat', 'unsat', 'unknown', 'terminated']:
            new_state = 'ended'
        self.state_tasks_dict[new_state].append(t)
        t.time_infos[new_state] = self.get_current_time()
        
    def update_task_pstate(self, t: Task, new_pstate: str):
        self.write_line_to_log(f'update-pstate {t.id} {new_pstate}')
        t.pstate = new_pstate
        self.pstate_tasks_dict[new_pstate].append(t)
        t.time_infos[new_pstate] = self.get_current_time()
    
    def check_simplifying_state(self, t: Task):
        if t.state == 'unsat':
            return False
        sta = self.check_simplifying_process(t.id, t.sp)
        if sta == 'simplifying':
            return True
        if sta == 'simplified':
            self.update_task_state(t, 'simplified')
            self.update_task_pstate(t, 'unpartitioned')
        return False
    
    def check_partitioning_pstate(self, t: Task):
        if t.pstate == 'solved':
            return False
        sta = self.check_partitioning_process(t.id, t.pp)
        if sta == 'partitioning':
            return True
        self.update_task_pstate(t, sta)
        if sta == 'partitioned':
            self.update_task_state(t.subtasks[0], 'generated')
            self.update_task_state(t.subtasks[1], 'generated')
        return False
    
    # True for still running
    def check_solving_state(self, t: Task):
        if t.state in ['unsat', 'unknown', 'terminated']:
            return False
        sta = self.check_solving_process(t.id, t.sp)
        if sta == 'solving':
            return True
            # if self.need_terminate(t):
            #     self.update_task_state(t, 'terminated')
            #     t.sp.terminate()
            #     return False
            # else:
            #     return True
        
        if sta == 'sat':
            self.result = 'sat'
            self.reason = t.id
            self.done = True
            self.write_line_to_log(f'sat-task {t.id}')
            return False
        
        t.reason = t.id
        if sta == 'unsat':
            self.update_task_state(t, 'unsat')
            if t.parent != None:
                self.push_up(t.parent, t.id)
            
            for st in t.subtasks:
                self.push_down(st, t.id)
        else:
            self.update_task_state(t, 'unknown')
            self.write_line_to_log(f'unknown-node {t.id} {sta}')
        
        return False
    
    def check_runnings_state(self):
        if self.solve_ori_flag and self.ori_task.state == 'solving':
            sta = self.check_solving_process(self.ori_task.id, self.ori_task.sp)
            if sta != 'solving':
                self.write_line_to_log(f'original-result {sta}')
                if sta in ['sat', 'unsat']:
                    self.result = sta
                    self.done = True
                    return
                self.ori_task.state = sta
                self.base_run_cnt -= 1
        
        still_solvings = []
        for t in self.state_tasks_dict['solving']:
            t: Task
            if t.state != 'solving':
                continue
            if self.check_solving_state(t):
                still_solvings.append(t)
            else:
                if self.done:
                    return
        self.state_tasks_dict['solving'] = still_solvings
        
        if len(self.tasks) > 0:
            root_task: Task = self.tasks[0]
            if root_task.state == 'unsat':
                self.result = 'unsat'
                self.reason = root_task.reason
                self.done = True
                self.write_line_to_log(f'unsat-root-task {root_task.reason}')
                return
        
        still_simplifyings = []
        for t in self.state_tasks_dict['simplifying']:
            t: Task
            if t.state != 'simplifying':
                continue
            if self.check_simplifying_state(t):
                still_simplifyings.append(t)
        self.state_tasks_dict['simplifying'] = still_simplifyings
        
        still_partitionings = []
        for t in self.pstate_tasks_dict['partitioning']:
            if self.check_partitioning_pstate(t):
                still_partitionings.append(t)
        self.pstate_tasks_dict['partitioning'] = still_partitionings
        
    def simplify_task(self, t: Task):
        cmd =  [self.partitioner_path,
                f'{self.output_dir_path}/tasks/task-{t.id}.smt2', 
                '--partition.output-dir', f'{self.output_dir_path}',
                '--partition.next-id', f'{t.id}',
                '--partition.simplify', '1',
            ]
        # os.system(' '.join(cmd))
        self.write_line_to_log('exec-command {}'.format(' '.join(cmd)))
        t.sp = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
        self.update_task_state(t, 'simplifying')
    
    def partition_task(self, t: Task):
        cmd =  [self.partitioner_path,
                f'{self.output_dir_path}/tasks/task-{t.id}-simplified.smt2', 
                '--partition.output-dir', self.output_dir_path,
                '--partition.next-id', f'{self.next_task_id}',
                '--partition.simplify', '0',
            ]
        self.write_line_to_log('exec-command {}'.format(' '.join(cmd)))
        t.pp = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
        self.update_task_pstate(t, 'partitioning')
        self.make_task(t.id)
        self.make_task(t.id)
    
    def solve_task(self, t: Task):
        cmd =  [self.solver_path,
                f'{self.output_dir_path}/tasks/task-{t.id}-simplified.smt2',
            ]
        self.write_line_to_log('exec-command {}'.format(' '.join(cmd)))
        t.sp = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
        self.update_task_state(t, 'solving')
    
    def get_running_num(self):
        return len(self.state_tasks_dict['simplifying']) \
             + len(self.pstate_tasks_dict['partitioning']) \
             + len(self.state_tasks_dict['solving']) \
             + self.base_run_cnt
    
    def get_unended_num(self):
        return len(self.tasks) \
             - len(self.state_tasks_dict['ended'])
    
    def simplify_generated_tasks(self):
        running_num = self.get_running_num()
        if running_num >= self.max_running_tasks:
            return
        still_generateds = []
        for t in self.state_tasks_dict['generated']:
            t: Task
            if t.state != 'generated':
                continue
            if running_num >= self.max_running_tasks:
                still_generateds.append(t)
            else:
                self.simplify_task(t)
                running_num += 1
                self.write_line_to_log(f'running: {self.get_running_num()}, unended: {self.get_unended_num()}')
        self.state_tasks_dict['generated'] = still_generateds
    
    def partition_unpartitioned_tasks(self):
        running_num = self.get_running_num()
        unended_num = self.get_unended_num()
        if running_num >= self.max_running_tasks \
           or unended_num >= self.max_unended_tasks:
            return
        
        still_unpartitioneds = []
        for t in self.pstate_tasks_dict['unpartitioned']:
            t: Task
            if t.pstate != 'unpartitioned':
                continue
            if running_num >= self.max_running_tasks \
               or unended_num >= self.max_unended_tasks:
                still_unpartitioneds.append(t)
            else:
                self.partition_task(t)
                running_num += 1
                unended_num += 2
                self.write_line_to_log(f'running: {self.get_running_num()}, unended: {self.get_unended_num()}')
        self.pstate_tasks_dict['unpartitioned'] = still_unpartitioneds
    
    def solve_simplified_tasks(self):
        running_num = self.get_running_num()
        if running_num >= self.max_running_tasks:
            return
        still_simplifieds = []
        for t in self.state_tasks_dict['simplified']:
            t: Task
            if t.state != 'simplified':
                continue
            if running_num >= self.max_running_tasks:
                still_simplifieds.append(t)
            else:
                self.solve_task(t)
                running_num += 1
                self.write_line_to_log(f'running: {self.get_running_num()}, unended: {self.get_unended_num()}')
        self.state_tasks_dict['simplified'] = still_simplifieds
    
    def solve_ori_task(self):
        # run original task
        self.ori_task = Task(-2, None, self.get_current_time())
        
        cmd =  [self.solver_path,
                self.input_file_path
            ]
        # print(' '.join(cmd))
        sp = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
        self.ori_task.sp = sp
        self.ori_task.state = 'solving'
        self.ori_task.time_infos['solving'] = self.get_current_time()
        self.base_run_cnt += 1
    
    def init_root_task(self):
        # self.write_line_to_log(f'cp {self.input_file_path} {self.output_dir_path}/tasks/task-0.smt2')
        os.system(f'cp {self.input_file_path} {self.output_dir_path}/tasks/task-0.smt2')
        self.make_task(-1)
        rt: Task = self.tasks[0]
        self.update_task_state(rt, 'generated')
        self.update_task_pstate(rt, 'not-ready')
    
    # def check_done(self):
    #     if self.done:
    #         return True
    #     if self.solve_ori_flag and self.ori_task.state == 'solving':
    #         return False
    #     if self.get_unended_num() > 0:
    #         return False
    #     return True
    
    def solve(self):
        if self.solve_ori_flag:
            self.solve_ori_task()
        self.init_root_task()
        while True:
            self.check_runnings_state()
            # if self.check_done():
            if self.done:
                return
            self.simplify_generated_tasks()
            self.partition_unpartitioned_tasks()
            self.solve_simplified_tasks()
            if self.get_current_time() >= self.time_limit:
                raise TimeoutError()
            if self.get_running_num() >= self.max_running_tasks:
                time.sleep(0.1)
        
    def sum_up(self, result, exec_time):
        with open(f'{self.output_dir_path}/result.txt', 'w') as f:
            f.write(f'{result}\n{exec_time}\n')
        
    def terminate_all(self):
        if self.solve_ori_flag:
            self.ori_task.sp.terminate()
        for t in self.tasks:
            t: Task
            if t.sp != None:
                t.sp.terminate()
            if t.pp != None:
                t.pp.terminate()
            
    def __call__(self):
        self.init()
        try:
            self.solve()
        except TimeoutError:
            self.result = 'timeout'
            self.write_line_to_log('timeout')
        except AssertionError:
            self.result = 'AssertionError'
            self.write_line_to_log('AssertionError')
        
        end_time = time.time()
        execution_time = end_time - self.start_time
        print(self.result)
        print(execution_time)
        
        self.sum_up(self.result, execution_time)
        self.terminate_all()
        self.clean_up()
        
if __name__ == '__main__':
    bvp = BV_Parti()
    bvp()
    
'''

'''
