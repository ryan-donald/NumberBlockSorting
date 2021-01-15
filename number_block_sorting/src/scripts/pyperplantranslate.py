#!/usr/bin/env python

import numpy as np
import os

class PyperPlanTranslation:

    defineStr = ""
    objStr = ""
    initStr = ""
    goalStr = ""

    #initializes the strings for the define statement, object statement, init statement, and goal statement for the task.pddl file used with the corresponding domain file
    def TranslateToPDDL(self,blockNumArr):

        self.defineStr = "(define (problem BLOCKS-" + str(blockNumArr.size) + "-0)\n"
        self.initialArr = np.copy(blockNumArr)
        np.sort(blockNumArr)
        self.objStr = "(: objects "
        for x in blockNumArr:
            self.objStr = self.objStr + str(x) + " "
        
        self.objStr = self.objStr + "- block)\n"
    
        self.initStr = "(:INIT "
        for x in blockNumArr:
            self.initStr = self.initStr + " (CLEAR " + str(x) + ") "
        
        for x in blockNumArr:
            self.initStr = self.initStr + "(ONTABLE " + str(x) + ") "
        
        self.initStr = self.initStr + "(HANDEMPTY))\n"

        self.goalStr = "(:goal (AND "

        for x in range(blockNumArr.size - 1):
            self.goalStr = self.goalStr + "(LEFT " + str(x) + " " + str(x+1) + ") "
        
        self.goalStr = self.goalStr + ") (HANDEMPTY))\n)"

    #creates the task.pddl file in the right format to be used with pyperplan using the strings created 
    def CreatePDDLFile(self):

        outF = open("sortTask01.pddl", 'w')

        outF.write(self.defineStr + "(:domain BLOCKS)\n" + self.objStr + self.initStr + self.goalStr)

        outF.close()

    def InterpretSolution(self):
        inF = open("/home/ryan/catkin_ws/src/NumberBlockSorting/number_block_sorting/src/scripts/sortTask01.pddl.soln", 'r')

        self.commands = inF.readlines()

        inF.close()

        



    #testing as the code for creating various arrays is not written
#if __name__ == '__main__':
    #test = PyperPlanTranslation()

    #arr = np.array([1,2,3,4])

    #test.TranslateToPDDL(arr)

    #test.CreatePDDLFile()



