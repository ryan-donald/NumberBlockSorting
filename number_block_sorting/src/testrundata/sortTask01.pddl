(define (problem BLOCKS-4-0)
(:domain BLOCKS)
(:objects 1 4 3 2 - block)
(:INIT  (CLEAR 1)  (CLEAR 2)  (CLEAR 3)  (CLEAR 4) (ONTABLE 1) (ONTABLE 2) (ONTABLE 3) (ONTABLE 4) (HANDEMPTY))
(:goal (AND (LEFT 1 2) (LEFT 1 3) (LEFT 1 4) (LEFT 2 3) (LEFT 2 4) (LEFT 3 4) (HANDEMPTY) ))
)

