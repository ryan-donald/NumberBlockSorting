#!/usr/bin/env python

import numpy as np
import os
import subprocess
import rospkg

class PyperPlanTranslation(object):

    defineStr = ""
    objStr = ""
    initStr = ""
    goalStr = ""

    def RunPlanner(self, blocks):
        
        self.TranslateToPDDL(blocks)
        self.CreatePDDLFile()
        self.ExecutePlanner()
        self.InterpretSolution()


    #initializes the strings for the define statement, object statement, init statement, and goal statement for the task.pddl file used with the corresponding domain file
    def TranslateToPDDL(self, blockNumArr):

        self.rospack = rospkg.RosPack()
        self.pkgPath = self.rospack.get_path('number_block_sorting')

        self.defineStr = "(define (problem BLOCKS-" + str(blockNumArr.size) + "-0)\n"
        self.initialArr = np.copy(blockNumArr)
        sortedArr = np.sort(blockNumArr)
        self.objStr = "(:objects "
        for x in blockNumArr:
            self.objStr = self.objStr + str(x) + " "
        
        self.objStr = self.objStr + "- block)\n"
    
        self.initStr = "(:INIT "
        for x in blockNumArr:
            self.initStr = self.initStr + " (CLEAR " + str(x) + ") "
        
        for x in blockNumArr:
            self.initStr = self.initStr + "(ONTABLE " + str(x) + ") "
        
        for x in range(4):

            for y in range(4):
                
                if (x < y):
                    self.initStr = self.initStr + "(LEFT " + str(self.initialArr[x]) + " " + str(self.initialArr[y]) + ") "
        
        
        self.initStr = self.initStr + "(HANDEMPTY))\n"

        self.goalStr = "(:goal (AND "

        for x in sortedArr:

            for y in sortedArr:

                if (x < y):
                    self.goalStr = self.goalStr + "(LEFT " + str(x) + " " + str(y) + ") "
        
        self.goalStr = self.goalStr + "(HANDEMPTY) ))\n)"

    #creates the task.pddl file in the right format to be used with pyperplan using the strings created 
    def CreatePDDLFile(self):
        
        outF = open(self.pkgPath + "/src/scripts/PyperPlanFiles/sortTask04.pddl", 'w')

        outF.write(self.defineStr + "(:domain BLOCKS)\n" + self.objStr + self.initStr + self.goalStr)

        outF.close()

    def InterpretSolution(self):

        inF = open(self.pkgPath + "/src/scripts/PyperPlanFiles/sortTask04.pddl.soln", 'r')

        self.commands = inF.readlines()

        inF.close()

    def ExecutePlanner(self):

        shellCommand = "pyperplan -H hff -s gbf " + self.pkgPath + "/src/scripts/PyperPlanFiles/domain.pddl " + self.pkgPath + "/src/scripts/PyperPlanFiles/sortTask04.pddl"

        subprocess.call(shellCommand, shell=True)



    #testing as the code for creating various arrays is not written
#if __name__ == '__main__':

    #prevDir = os.getcwd()

    #print(prevDir)
    #print(os.listdir("~/catkin_ws/src/NumberBlockSorting/number_block_sorting/src/scripts/PyperPlanFiles"))
    #os.chdir('~/catkin_ws/src/NumberBlockSorting/number_block_sorting/src/scripts/PyperPlanFiles/')
    #subprocess.call("cd ~/catkin_ws/src/NumberBlockSorting/number_block_sorting/src/scripts/PyperPlanFiles", shell=True)

    #currDir = os.getcwd()

    #print(currDir)

    #arr = np.array([1,2,3,4])

    #test = PyperPlanTranslation()

    #test.RunPlanner(arr)

    #test.TranslateToPDDL(arr)

    #test.CreatePDDLFile()

    #test.ExecutePlanner()

    #test.InterpretSolution()

    #print(test.commands)



