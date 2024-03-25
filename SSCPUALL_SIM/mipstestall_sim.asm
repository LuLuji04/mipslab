##########################################################################################
#
# Designer:   Qin Liu
#
# Description:
# As part of the project of Computer Organization Experiments, Wuhan University
# In spring 2024
#
##########################################################################################

# Test the MIPS processor in simulation
# sll
#

#       Assembly                  Description           Instr   Address Machine
main:   addi $2, $0, 4          # initialize $2 = 4     00      0       20020005
        #sll  $5, $2, 2          # $2 << 2
        #addi $4, $0, 2
        #srlv  $6, $2, $4

        #nor $4, $2, $2

        #lui $4, 0x0001
        #lui $5, 0xf000

        #addi $5, $0, 2
        #slti $6, $5, 4

        #add $3,$2,$2
        #bne $3,$2,main

        #addi $3,$0,0
        #jr $3

        #addi $3,$0,4
        #jalr $4,$3

        sw $2 0x184($0)

loop:   j    loop               # dead loop             12      48      08000012

# $6 = 14

