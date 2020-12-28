#! /usr/bin/env python

#Ryan Donald 11/20/2020. 
# sorting blocks based on the number on the block.
# stores the initial and final positions of the blocks as a string in the text file blockposdata.txt

#class used for sorting
import numpy as np

class Sorting():

    blockPos= np.empty(3)
    dataFile = open("~/catkin_ws/src/NumberBlockSorting/number_block_sorting/data/blockposdata.txt", 'w')

    def SortingBlocks(self, blockArr):
        self.blockPos = np.copy(blockArr)
        self.StoreInitialPos(blockArr)

        np.sort(blockArr)
        
        self.StoreFinalPos(blockArr)
        
        return blockArr
    
    def StoreInitialPos(self, blockArr):
        self.dataFile.write("Initial Block Position: \n", 'a')
        self.dataFile.write(np.array2string(self.blockPos)+"\n")
        
        return

    def StoreFinalPos(self,blockArr):
        self.dataFile.write("Final Block Position: \n", 'a')
        self.dataFile.write(np.array2string(self.blockPos) + "\n\n\n")
        self.dataFile.close()
        return