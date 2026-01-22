# Simple interactive dungeon crawler
# This code was written for the AI courses in computer engineering at Université de Sherbrooke
# Author : Audrey Corbeil Therrien

from swiplserver import PrologMQI
from Games2D import *
import A_star
# import ctypes

if __name__ == '__main__':
    # Niveau 0 - sans obstacle - 'assets/Mazes/mazeMedium_0'
    # Niveau 1 - avec obstacles - 'assets/Mazes/mazeMedium_1'
    # Niveau 2 - avec obstacles, portes et un ennemi - 'assets/Mazes/mazeMedium_2'
    # Niveau 3 - avec obstacles, portes et plusieurs ennemis - 'assets/Mazes/mazeMedium_2'

    # If the window is too big for the screen, uncomment the following line
    # The effect of fixed pixel algorithms has NOT BEEN TESTED. PROCEED WITH CAUTION.
    # ctypes.windll.user32.SetProcessDPIAware()
    with PrologMQI() as mqi_file:
        with mqi_file.create_thread() as prolog_thread:
            prolog_thread.query("[prolog/porte]")
            theAPP = App('assets/Mazes/mazeMedium_2', prolog_thread)
            theAPP.on_execute()

