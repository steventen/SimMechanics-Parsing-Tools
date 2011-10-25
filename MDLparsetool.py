from pyparsing import *
import re
import string

"""
This tool is for parsing MDL file, which is created directly under Matlab's SimMechanics system.

Methods:
	getParam(mdlSys, attriName, ex = 1)
	findBlock(mdlSys, obj, attriName, keyVal)
	getWorkingFrame(mdlSys, value)
	getConnection(mdlSys, jointList)

Take a look at main() part to learn how to use them

Author: Cong (Steven) Yue
E-mail: steventenie@gmail.com

Credits:
Code for parsing mdl file into nest list was writtern by Kjell Magne Fauske,
and most of his code is based on the json parser example distributed with
pyparsing. The code in jsonParser.py was written by Paul McGuire

"""

## Code from Kjell Magne Fauske #############

# A high level grammar of the Simulink mdl file format
SIMULINK_BNF = """
object {
     members
}
members
    variablename  value
    object {
        members
    }
variablename

array
    [ elements ]
matrix
    [elements ; elements]
elements
    value
    elements , value
value
    string
    doublequotedstring
    float
    integer
    object
    array
    matrix
"""


# parse actions
def convertNumbers(s,l,toks):
    """Convert tokens to int or float"""
    # Taken from jsonParser.py
    n = toks[0]
    try:
        return int(n)
    except ValueError, ve:
        return float(n)

def joinStrings(s,l,toks):
    """Join string split over multiple lines"""
    return ["".join(toks)]


def mdlParser(mdlFilePath):
	mdldata = open(mdlFilePath,'r').read()
	# Define grammar

	# Parse double quoted strings. Ideally we should have used the simple statement:
	#    dblString = dblQuotedString.setParseAction( removeQuotes )
	# Unfortunately dblQuotedString does not handle special chars like \n \t,
	# so we have to use a custom regex instead.
	# See http://pyparsing.wikispaces.com/message/view/home/3778969 for details. 
	dblString = Regex(r'\"(?:\\\"|\\\\|[^"])*\"', re.MULTILINE)
	dblString.setParseAction( removeQuotes )
	mdlNumber = Combine( Optional('-') + ( '0' | Word('123456789',nums) ) +
						Optional( '.' + Word(nums) ) +
						Optional( Word('eE',exact=1) + Word(nums+'+-',nums) ) )
	mdlObject = Forward()
	mdlName = Word('$'+'.'+'_'+alphas+nums)
	mdlValue = Forward()
	# Strings can be split over multiple lines
	mdlString = (dblString + Optional(OneOrMore(Suppress(LineEnd()) + LineStart()
				 + dblString)))
	mdlElements = delimitedList( mdlValue )
	mdlArray = Group(Suppress('[') + Optional(mdlElements) + Suppress(']') )
	mdlMatrix =Group(Suppress('[') + (delimitedList(Group(mdlElements),';')) \
				  + Suppress(']') )
	mdlValue << ( mdlNumber | mdlName| mdlString  | mdlArray | mdlMatrix )
	memberDef = Group( mdlName  + mdlValue ) | Group(mdlObject)
	mdlMembers = OneOrMore( memberDef)
	mdlObject << ( mdlName+Suppress('{') + Optional(mdlMembers) + Suppress('}') )
	mdlNumber.setParseAction( convertNumbers )
	mdlString.setParseAction(joinStrings)
	# Some mdl files from Mathworks start with a comment. Ignore all
	# lines that start with a #
	singleLineComment = Group("#" + restOfLine)
	mdlObject.ignore(singleLineComment)
	mdlparser = mdlObject
	result = mdlparser.parseString(mdldata)
	return result
## end of his code #############

def getParam(mdlSys, attriName, ex = 1):
	""" 
	Find the value of a specified attribute in the mdl system sequence
		If ex = 1 : list style would be like: [Block Name, value]
		If ex = 0 : list style would only be the value of the attribute
	"""
	result = []
	for syslist in mdlSys:
		if syslist[0]=='System':
			for blocklist in syslist:
				if blocklist[0]=='Block':
					for item in blocklist:
						if item[0]== attriName:
							if ex == 1:
								ans = [blocklist[2][1], item[1]]
							elif ex == 0:
								ans = item[1]
							result.append(ans)
	
	if result == []:
		for blocklist in mdlSys:
			if blocklist[0]=='Block':
				for item in blocklist:
					if item[0]== attriName:
						if ex == 1:
							ans = [blocklist[2][1], item[1]]
						elif ex == 0:
							ans = item[1]
						result.append(ans)
	if result == []:
		for blocklist in mdlSys:
			for item in blocklist:
				if item[0]== attriName:
					if ex == 1:
						ans = [blocklist[2][1], item[1]]
					elif ex == 0:
						ans = item[1]
					result.append(ans)
	if result == []:
		for item in mdlSys:
			if item[0]== attriName:
				if ex == 1:
					ans = [blocklist[2][1], item[1]]
				elif ex == 0:
					ans = item[1]
				result.append(ans)
	return result
		

def findBlock(mdlSys, obj, attriName, keyVal):
	"""
	Find all the Blocks that has an keyVal in an Attribute
	"""
	result = []
	for syslist in mdlSys:
		if syslist[0]=='System':
			for blocklist in syslist:
				if blocklist[0]== obj:  #For Body or Joint use 'Block', For line use 'Line'
					for item in blocklist:
						if item[0]== attriName and item[1]==keyVal:
							ans = blocklist
							
							result.append(ans)
	return result
	
	
def getWorkingFrame(mdlSys, value):
	"""
	Speical used for parsing the WorkingFrames Attribute of Body Blocks
	   Can Get the CG and CS systems
	"""
	wPattern = getParam(mdlSys, value)
	#print 'wPattern'
	#pprint(wPattern)
	#heading='^CS\d$' # it will miss CS10,CS11,...
	heading='^CS'
	if value == 'CG':
		heading='^CG$'
	bodyname = []
	bodyList = []
	for i in wPattern:
		bodyname = i[0]
		CS_param = i[1].split(r'$')
		CSlist=[bodyname,[]]
		for word in xrange(len(CS_param)):
			if re.search(heading,CS_param[word]):
				print 'word=',word,
				print 'CS_param=',  CS_param
				print bodyname,'CS',CS_param[word]
				pos        = CS_param[word+1] #Position
				origin     = CS_param[word+2] #Translated from origin of 
				axes       = CS_param[word+3] #Components in axes of 
				pos_units  = CS_param[word+4] #Position Units
				ori        = CS_param[word+5] #Oritention
				convention = CS_param[word+6] #Specified Using Convention
				ori_units  = CS_param[word+7] #Orientation Units
				rel_CS     = CS_param[word+8] #Relative CS
				#print bodyname, CS_param[word], pos, origin, axes, pos_units, ori, convention, ori_units, rel_CS
				CSlist[1].append([CS_param[word], pos, ori, rel_CS])
				word = word + 11
			if word+11 > len(CS_param):
				break
		bodyList.append(CSlist)
	return bodyList


def getConnection(mdlSys, jointList):
	lineList = findBlock(mdlSys,'Line','LineType','Connection')
	jointConnList = []
	for jointName in jointList:
		jointConn = [jointName]
		jointBlockList = findBlock(mdlSys,'Block', 'Name', jointName)
		jointFramesList = getParam(jointBlockList, 'PrimitiveProps', ex = 0)
		jointFrames = jointFramesList[0].split('$')
		joint_Axis = jointFrames[2]
		joint_TypeName=getParam(jointBlockList,'SourceType',0)[0]
		jointConn.append(joint_TypeName)
		jointConn.append(joint_Axis)
		joint_relCS = jointFrames[1]
		jointConn.append(joint_relCS)
		
		for blocklist in lineList:
			for item in blocklist:
				if item[0] == 'DstBlock' and item[1] == jointName:
					baseName = getParam(blocklist, 'SrcBlock', ex = 0)
					basePortList = getParam(blocklist, 'SrcPort', ex = 0)
					basePortVal = basePortList[0]
					basePort = basePortVal.split('Conn')
					baseBlock = findBlock(mdlSys,'Block', 'Name', baseName[0])
					
					bconnCSValList = getParam(baseBlock, basePort[0]+'ConnTagsString', ex = 0)
					if bconnCSValList == []:
						bconnCSValList = ['None']
					
					bconnCSVal = bconnCSValList[0]
					numCS = eval(basePort[-1])
					baseConn = bconnCSVal.split('|')[numCS - 1]  #Find the digital number of CS which is the base to connect to the joint
					
					jointConn.append([baseName[0], baseConn])
				
				if item[0] == 'SrcBlock' and item[1] == jointName:
					followerName = getParam(blocklist, 'DstBlock', ex = 0)
					
					followerPortList = getParam(blocklist, 'DstPort', ex = 0)  #Note: getParam() returns a list
					followerPortVal = followerPortList[0]
					followerPort = followerPortVal.split('Conn')
					followerBlock = findBlock(mdlSys,'Block', 'Name', followerName[0])
					
					fconnCSValList = getParam(followerBlock, followerPort[0]+'ConnTagsString', ex = 0)
					if fconnCSValList == []:
						fconnCSValList = ['None']
						
					fconnCSVal = fconnCSValList[0]
					numCS = eval(followerPort[-1])
					followerConn = fconnCSVal.split('|')[numCS - 1]  #Find the digital number of CS which is the base to connect to the joint
					jointConn.append([followerName[0], followerConn])
				
		jointConnList.append(jointConn)						

	return jointConnList

#def getGraphicFileName(mdlSys, value):
#	wPattern = getParam(mdlSys, value)
#	#print 'wPattern'
#	#pprint(wPattern)
#	#heading='^CS\d$' # it will miss CS10,CS11,...
#	heading='^CS'
#	if value == 'CG':
#		heading='^CG$'
#	bodyname = []
#	bodyList = []
#	for i in wPattern:
#		bodyname = i[0]
	
#def parseMatrix(stringMatrix):
#	"""Get the real numbers from a string that contains a matrix-like structure"""
#	#print stringMatrix
#	try:
#		numMatrix = eval(stringMatrix.translate(string.maketrans(' ;',',,')))
#	except:
#		numMatrix = eval(stringMatrix)
#	return numMatrix

#mmt = parseMatrix('-122.558,14.7511,2.5')
#print mmt
#print mmt[1]
#def getCG(sys):
#	wPattern = get_param(sys, 'CG')
#	bodyname = []
#	bodyListCG = []
#	for i in wPattern:
#		bodyname = i[0]
#		CG_param = i[1].split(r'$')
#		CGlist=[bodyname]
#		for word in range(len(CG_param)):
#			if re.search('^CG$',CG_param[word]):
#				pos        = CG_param[word+1] #Position
#				origin     = CG_param[word+2] #Translated from origin of 
#				axes       = CG_param[word+3] #Components in axes of 
#				pos_units  = CG_param[word+4] #Position Units
#				ori        = CG_param[word+5] #Oritention
#				convention = CG_param[word+6] #Specified Using Convention
#				ori_units  = CG_param[word+7] #Orientation Units
#				rel_CG     = CG_param[word+8] #Relative CG
#				
#				print bodyname, CG_param[word], pos, origin, axes, pos_units, ori, convention, ori_units, rel_CG
#				CGlist.append([CG_param[word], pos, ori, rel_CG])
#				word = word + 11
#		bodyListCG.append(CSlist)
#	return bodyListCG
	
#def find_object(sys, value)
#def find_system()	
#	def searchIn(seq):
#		for item in seq:
#			if isinstance(item, list):
#				searchIn(item)
#			else:
#				if item == value:
#					m=m+1
#					print m
			

#for index, item in enumerate(seq):
#		print 1
#		if value in item:
#			return index, item


def main():
	import os
	from pprint import pprint
	filePath = os.path.join(os.getcwd(),'testExample','fourBar.mdl')
	#testdata = open('AIRCRAFT_ENGINE.mdl','r').read()
	result = mdlParser(filePath)
	mdldata = result.asList()
	#mdldata = result
	
	bodyListCG = getWorkingFrame(mdldata, 'CG')
	bodyListCS = getWorkingFrame(mdldata, 'WorkingFrames')
	print 'bodyListCG'
	pprint(bodyListCG)
	JointList = findBlock(mdldata,'Block','DialogClass','JointBlock')
	JointNameList = getParam(JointList,'Name',0)
	ConnList = getConnection(mdldata, JointNameList)
	print 'ConnList'
	pprint(ConnList)
	print 'ConnList[1]'
	pprint(ConnList[1])
	print 'ConnList[1][1]'
	pprint(ConnList[1][1])
	fListT = open('fourBarTemplete.txt','wb')
	import pickle
	mechInfo = [bodyListCS,bodyListCG,JointList,ConnList]
	pickle.dump(mechInfo,fListT) # directly store an object into file
	fListT.close()
	readTemp = open('fourBarTemplete.txt','rb')
	rListT = pickle.load(readTemp)
#	print 'rListT',rListT
#	rbodyListCS = rListT[0]
#	rJointList = rListT[1]
#	rConnList = rListT[2]
#	print 'rbodyListCS', rbodyListCS
#	print 'rJointList', rJointList

### test stuff
if __name__ == '__main__':
	main()

