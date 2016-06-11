import cma
import numpy as np
import pickle
import os
import rospy


# Hack to check feasibility as instance method fails to pickle
def check_feasible(params, fitness):
    if fitness is not None:
        if any([v > 1000 for v in fitness]):
            rospy.logwarn('Const Check: Failed fitness value {}'.format(fitness))
            return False
        else:
            rospy.logwarn('Const Check: Skip feasibility, good reward')
            return True
    return True


def hans_setup_cmaes(sigma, initial_params, constraint_func=False, checkpoint=False, filename=False):
    opts = cma.CMAOptions()
    if constraint_func:
        opts.set('is_feasible', check_feasible)

    # opts.set('bounds', [0, 10])
    # opts.set('boundary_handling', 'BoundTransform')

    if checkpoint and filename:
        es = pickle.load(open(filename, 'rb'))
    else:
        es = cma.CMAEvolutionStrategy(initial_params, sigma, opts)

    return es


def hans_run_cmaes(es, eval_function, pkl_location, log_iters=1):
    while not es.stop():
        rospy.loginfo("**********************************************")
        rospy.loginfo("   ******* Current iteration {} ********".format(es.countiter))
        rospy.loginfo("**********************************************")
        X, fit = es.ask_and_eval(eval_function)
        es.tell(X, fit)  # pass on fitness values
        es.logger.add()
        es.disp(20)  # print every 20-th iteration
        # if es.countiter % log_iters == 0:
        #     try:
        #         es.logger.plot()
        #     except:
        #         pass
        pickle.dump(es, open(os.path.join(pkl_location, 'cma_{:05d}.pkl'.format(es.countiter)), 'wb'))

    print('Terminated on ' + str(es.stop()))
    pickle.dump(es, open(os.path.join(pkl_location, 'cma_final.pkl'), 'wb'))
    cma.pprint(es.result())
