##########################################################################################
#
# Designer:   Qin Liu
#
# Description:
# As part of the project of Computer Organization Experiments, Wuhan University
# In spring 2024
# MIPS assembly code for sorting the sid digital numbers in the descending order for simulation.
#
##########################################################################################


##########################################################################################
# C pseudocode for descending order sorting the sid digital numbers.
##########################################################################################
#   sortedsid = sid;
#   mask0 = 0x0f;
#   each iteration selects the smallest digital number from the unprocessed digital numbers
#   for (int i= 0; i < 8; i++) {
#       a = sortedsid & mask0;
#       a = a >> (4 * i);
#       mask1 = mask0 << 4;
#       bestj = i;
#       tmpMin = a;
#     for (int j = i + 1; j < 8) {
#       b = sortedsid & mask1;
#       b = b >> (4 * j);
#       if (tmpMin > b) {
#          tmpMin = b;
#          bestj = j;
#       }
#       mask1 = mask1 << 4;
#     }
#     if (a > tmpMin) {
#         mask1 = 0x0f;
#         bestj4 = bestj << 2;
#         mask1 = mask1 << bestj4
#         mask2 = mask0 | mask1;
#         mask2 = ~mask2;
#         sortedsid = sortedsid & mask2;
#         tmpMax = tmpMax << (4 * i);
#         sortedsid = sortedsid | tmpMax;
#         a = a << bestj4;
#         sortedsid = sortedsid | a;
#       }
#     mask0 = mask0 << 4;
#   }
##########################################################################################
# the uasge of the registers
##########################################################################################
# mem[0x180], student id.
# mem[0x184], sorted student id
# $1, partially sorted student number / the address of switch
# $2, the outer loop variable i / the address of seg7
# $3, the inner loop variable j / the switch input
# $4, mask0
# $5, mask1
# $6, mask2
# $7, a
# $8, b
# $9, 4 * i
# $10, 4 * j
# $11, N = 8
# $12, bestj
# $13, tmpMin
# $14, compare result
##########################################################################################
# MIPS assembly language program for sorting the student id.
# The following instructions are used.
# add and nor or sll sllv srlv slt
# addi andi ori lui
# beq bne
# j jal jr
# lw sw
# sub sltu
# slti srl jalr
##########################################################################################
      lui     $2, 0x0218          #  high halfword the student id (last 8 digitals), use your own student id. instead!
      ori     $2, $2, 0x1020      #  low halfword of the student id (last 8 digitals)
      sw      $2, 0x180($0)       # store the original sid at data memory
      addi    $11, $0, 8          #  the size of sid, N = 8
      lw      $1, 0x180($0)       # $1 = [0x180] = sid
      add     $2, $0, $0          #  the outer loop variable initilization, i = 0,
      addi    $4, $0, 0x0f        #  mask0 = 0xf
loop1:
      and     $7, $1, $4          #  a = sortedsid & mask0, get the BCD to be processed
      sll     $9, $2, 2           #  (4 * i)
      srlv    $7, $7, $9          #  a = a >> (4 * i), shift the BCD to the LSB 4 bits
      sll     $5, $4, 4           #  mask1 = mask0 << 4
      add     $12, $2, $0         #  bestj = i, remmember the position of the smallest BCD in this loop
      add     $13, $7, $0         #  tmpMin = a, remember the smallest BCD in this loop
      addi    $3, $2, 1           #  j = i + 1, the inner loop variable initilization, j = i + 1
loop2:
      beq     $3, $11, checkswap  #  to check if j == 8
      and     $8, $1, $5          #  b = sortedsid & mask1
      sll     $10, $3, 2          # (4 * j)
      srlv    $8, $8, $10         #  b = b >> (4 * j), shift the BCD to the LSB 4 bits
      slt     $14, $8, $13        #  b-tmpMin
      beq     $14, $0, incrLoop2  #  if (tmpMin <= b), increase j
      add     $13, $8, $0         #  tmpMin = b, remember the smallest BCD in this loop
      add     $12, $3, $0         #  bestj = j, remmember the position of the smallest BCD in this loop
incrLoop2:
      sll     $5, $5, 4           #  mask1 = mask1 << 4
      addi    $3, $3, 1           #  j = j + 1
      j       loop2
checkswap:
      slt     $14, $2, $12        #  to check if the position of the smallest BCD in the this loop has been changed
      beq     $14, $0, incrLoop1
      jal     swap
incrLoop1:
      sll     $4, $4, 4           #  mask0 = mask0 << 4
      addi    $2, $2, 1           #  i = i + 1
      bne     $2, $11, loop1      #  to check if i <> 8

############# test the unused instrctions: sub, slti, sltu, srl, jalr#####################
## The logic of this part seems weird, and some instrutions are deliberately used to test them
## The correct execution should end with dead loop of result
      jal     test1
      srl     $0, $0, 0
      sll     $0, $0, 0
test1:
      addi    $15, $31, 68
      addi    $31, $0, 8
      sub     $15, $15, $31       #  put the address of label result in $15
      sll     $15, $15, 5
      srl     $15, $15, 5
      slti    $16, $15, -1
      bne     $16, $0, error1     # error1, test slti
      lui     $17, 0xffff
      sltu    $16, $15, $17
      beq     $16, $0, error1     # error1, test sltu
      jalr    $16, $15            # jump to result, test jalr, jump to $15, write pc+4 to $16
      srl     $0, $0, 0
      sll     $0, $0, 0
result:
      sw      $1, 0x184($0)       #  [0x184] = sortedsid
      lui     $2, 0xffff          #  $2 = 0xffff0000
      addi    $15, $16, 8         #  put the address of label result in $15
      jalr    $15                 # jump to result, dead loop, test jalr, jump to $15, write pc+4 to $31
##########################################################################################

swap:                             #  change the nibble at i with the nibble at bestj
      addi    $5, $0, 0x0f
      sll     $10, $12, 2         #  4 * bestj
      sllv    $5, $5, $10         #  mask1 = mask (4 * bestj)
      or      $6, $4, $5          #  mask2 = mask0 | mask1
      nor     $6, $6, $0          #  mask2 = ~mask2
      and     $1, $1, $6          #  sortedsid = sortedsid & mask2
      sllv    $8, $13,$9          #  tmpmax = tmpmax << (4*i)
      or      $1, $1, $8          #  sortedsid = sortedsid | tmpmax
      sllv    $7, $7, $10         #  a = a << (4 * bestj)
      or      $1, $1, $7          #  sortedsid = sortedsid | a
      jr      $31
error1:
      sw      $0, 0x184($0)       #  [0x184] = 0
      j       error1