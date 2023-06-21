import random
#increace count
#select catagory
#exempt proseeding catagories that use the same muscle group (including catagory selected)
#select exercise
#remove selected exercise from list
#find next exercise with exemptions in place
#select next catagory and exercise
#replace old exemption with new ones
#remove selected exercise from list
#----------------------------------------------------------------


#count number of exercises completed
count = 0 

#Catagory exemption
def exemption(num):
    ex0 = [0]
    ex1 = [1,7]
    ex2 = [2,3]
    ex3 = [3,2]
    ex4 = [4,5,6]
    ex5 = [5,6,4]
    ex6 = [6,5,6]
    ex7 = [7,1]
    exArr = [ex0,ex1,ex2,ex3,ex4,ex5,ex6,ex7]
    
    
    exemptLst.clear()

    for x in exArr[num]:
        exemptLst.append(x)

def rand():
     exemptLst= []

     catagory = random.randint(0,7) 
     print(catagory)
     return(catagory)


#store values to exempt for subsequent round in the exemptLst
while count<20:
    rand()
 
    for x in exemptLst:
        a = x
        if catagory == a:
            print("invalid choice")

        else:


count = count+1
