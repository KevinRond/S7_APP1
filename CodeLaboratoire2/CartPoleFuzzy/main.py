# Université de Sherbrooke
# Code for Artificial Intelligence module
# Adapted by Audrey Corbeil Therrien, Simon Brodeur

# Source code
# Classic cart-pole system implemented by Rich Sutton et al.
# Copied from http://incompleteideas.net/sutton/book/code/pole.c
# permalink: https://perma.cc/C9ZM-652R

# NOTE : The print_state function of the FuzzyController may need
# to be updated with the latest version, if you encounter the error, the fix is 
# available on github
# https://github.com/scikit-fuzzy/scikit-fuzzy/blob/master/skfuzzy/control/controlsystem.py
# Lines 514-572 from github replace lines 493-551 in the 0.4.2 2019 release

import gym
import time
from cartpole import *
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import matplotlib.pyplot as plt


def createFuzzyController():
    # TODO: Create the fuzzy variables for inputs and outputs.
    # Defuzzification (defuzzify_method) methods for fuzzy variables:
    #    'centroid': Centroid of area
    #    'bisector': bisector of area
    #    'mom'     : mean of maximum
    #    'som'     : min of maximum
    #    'lom'     : max of maximum
    angle = ctrl.Antecedent(np.linspace(-1, 1, 1000), 'angle')
    # ant2 = ctrl.Antecedent(np.linspace(-1, 1, 1000), 'input2')
    force = ctrl.Consequent(np.linspace(-10, 10, 1000), 'force', defuzzify_method='centroid')

    # Accumulation (accumulation_method) methods for fuzzy variables:
    #    np.fmax
    #    np.multiply
    force.accumulation_method = np.fmax

    # TODO: Create membership functions
    # To fix your specific code:
    angle['gauche'] = fuzz.trapmf(angle.universe, [-1, -1, -0.1, 0])
    angle['centre'] = fuzz.trapmf(angle.universe, [-0.1, -0.02, 0.02, 0.1])
    angle['droite'] = fuzz.trapmf(angle.universe, [0, 0.1, 1, 1])

    # ant2['membership1'] = fuzz.trapmf(ant2.universe, [-1, -0.5, 0.5, 1])

    force['gauche'] = fuzz.trapmf(force.universe, [-10, -10, -0.25, 0])
    force['centre'] = fuzz.trapmf(force.universe, [-0.25, -0.1, 0.1, 0.25])
    force['droite'] = fuzz.trapmf(force.universe, [0, 0.25, 10, 10])

    # TODO: Define the rules.
    rules = [
        ctrl.Rule(angle['gauche'], force['gauche']),
        ctrl.Rule(angle['centre'], force['centre']),
        ctrl.Rule(angle['droite'], force['droite'])
    ]

    # Conjunction (and_func) and disjunction (or_func) methods for rules:
    #     np.fmin
    #     np.fmax
    for rule in rules:
        rule.and_func = np.fmin
        rule.or_func = np.fmax

    system = ctrl.ControlSystem(rules)
    sim = ctrl.ControlSystemSimulation(system)
    return sim


# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    # Create the environment and fuzzy controller
    env = CartPoleEnv("human")
    fuzz_ctrl = createFuzzyController()

    # Display rules
    print('------------------------ RULES ------------------------')
    for rule in fuzz_ctrl.ctrl.rules:
        print(rule)
    print('-------------------------------------------------------')

    # Display fuzzy variables
    for var in fuzz_ctrl.ctrl.fuzzy_variables:
        var.view()
    plt.show()

    VERBOSE = False

    for episode in range(10):
        print('Episode no.%d' % (episode))
        env.reset()

        isSuccess = True
        action = np.array([0.0], dtype=np.float32)
        for _ in range(100):
            env.render()
            time.sleep(0.01)

            # Execute the action
            observation, _, done, _ = env.step(action)
            if done:
                # End the episode
                isSuccess = False
                break

            # Select the next action based on the observation
            cartPosition, cartVelocity, poleAngle, poleVelocityAtTip = observation

            # TODO: set the input to the fuzzy system
            fuzz_ctrl.input['angle'] = poleAngle
            # fuzz_ctrl.input['input2'] = 0

            fuzz_ctrl.compute()
            if VERBOSE:
                fuzz_ctrl.print_state()

            # TODO: get the output from the fuzzy system
            force = fuzz_ctrl.output['force']

            action = np.array(force, dtype=np.float32).flatten()

    env.close()
