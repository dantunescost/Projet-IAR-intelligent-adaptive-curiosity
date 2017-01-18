import operator

DELAY = 3

l1=[[1,2,5],[2,3,4],[3,1,20]]
l1.sort(key=operator.itemgetter(0))
l2=[0,1,2,3,4,5]
print l2[:2]
print l2[2:]
