from xml.dom import minidom

def getText(nodelist):
    rc = ""
    for node in nodelist:
        if node.nodeType == node.TEXT_NODE:
            rc = rc + node.data
    return rc

class XMLparsetool:
	def __init__(self, xmlFilePath):
		self.__xmlParsedFile = minidom.parse(xmlFilePath)

	def getCSList(self, whichList='CS'):
		"""
		Get the List of CG and CS for bodies
		The result formate would be like this:
			
		bodyListCS:
		[['PRT0001',
		  [['CS1',
		    '[-145.017 -110.606 5]',
		    '[-0.319989 0 0.947421;0.947421 0 0.319989;0 1 0]',
		    'WORLD'],
		   ['CS2', '[-212.558 14.7511 5]', '[1 0 0;0 1 0;0 0 1]', 'WORLD'],
		   ['CS3', '[-228.557 62.1221 10]', '[1 0 0;0 1 0;0 0 1]', 'WORLD']]],
		"""
		xmlfile = self.__xmlParsedFile
		
		#bodyListCG = []
		bodyListCS = []
		for body in xmlfile.getElementsByTagName('Body'):
			#bodyCGinfo = []
			bodyCSinfo = []
			#bodyCGList = []
			bodyCSList = []
			bodynameNode = body.getElementsByTagName('name')[0]
			bodyname = getText(bodynameNode.childNodes).strip('"')
			#bodyCGinfo.append(bodyname)
			bodyCSinfo.append(bodyname)
			frame = body.getElementsByTagName('Frame')
			for frameNo in xrange(len(frame)):
				#CGList=[]
				CSList=[]
				CSnameNode = frame[frameNo].getElementsByTagName('name')[0]
				CSname = getText(CSnameNode.childNodes).strip('"')
				CSpos = getText(frame[frameNo].getElementsByTagName('position')[0].childNodes).strip('"')
				CSposstring = '[' + CSpos + ']'
				CSori = getText(frame[frameNo].getElementsByTagName('orientation')[0].childNodes).strip('"')
				CSoristring = '[' + CSori + ']'
				ref_CS = getText(frame[frameNo].getElementsByTagName('positionOrigin')[0].childNodes).strip('"')
				if whichList == 'CG' and frameNo == 0:
					CSList.append(CSname)
					CSList.append(CSposstring)
					CSList.append(CSoristring)
					CSList.append(ref_CS)
					bodyCSList.append(CSList)
				elif whichList == 'CS'and frameNo > 0:
					CSList.append(CSname)
					CSList.append(CSposstring)
					CSList.append(CSoristring)
					CSList.append(ref_CS)
					bodyCSList.append(CSList)
			#bodyCGinfo.append(bodyCGList)
			bodyCSinfo.append(bodyCSList)
			#bodyListCG.append(bodyCGinfo)
			bodyListCS.append(bodyCSinfo)
		return bodyListCS

	def getConnList(self):
		"""
		Get the Connection of bodies and joints.
		The formate would be like this:
			
		ConnList:
		[['Revolute','JointType', '[0,0,-1]', 'WORLD', ['PRT0004', 'CS3'], ['PRT0001', 'CS2']], 
		['Revolute1', 'JointType','[0,0,-1]', 'WORLD', ['PRT0004', 'CS4'], ['PRT0003', 'CS2']], 
		['Revolute2', 'JointType','[0,0,-1]', 'WORLD', ['PRT0001', 'CS3'], ['PRT0002', 'CS2']], 
		['Revolute3', 'JointType','[0,0,-1]', 'WORLD', ['PRT0002', 'CS3'], ['PRT0003', 'CS3']], 
		['Weld', 'JointType','[0.57735,0.57735,0.57735]', 'WORLD', ['RootPart', 'CS2'], ['PRT0004', 'CS2']], 
		['Weld1', 'JointType','[0.57735,0.57735,0.57735]', 'WORLD', ['RootGround', 'None'], ['RootPart', 'CS3']]]

		"""
		xmlfile = self.__xmlParsedFile		
		ConnList = []
		baseInfoList = []
		followerInfoList = []
		jointTypeSpace = []
		jointNodeList = xmlfile.getElementsByTagName('SimpleJoint')
		for joint in xrange(len(jointNodeList)):
			jointPrimitive = jointNodeList[joint].getElementsByTagName('Primitive')[0]
			jointTypeNode = jointPrimitive.getElementsByTagName('name')[0]
			jointType = getText(jointTypeNode.childNodes).strip('"').capitalize()
			jointReferenceNode = jointPrimitive.getElementsByTagName('referenceFrame')[0]
			jointReference = getText(jointReferenceNode.childNodes).strip('"')
			jointAxisStringNode = jointPrimitive.getElementsByTagName('axis')[0]
			jointAxisString = getText(jointAxisStringNode.childNodes).strip('"')
			jointAxis = '[' + jointAxisString + ']'
			
			# name the joint accoording to the sequence of its appear, 1st weld named weld, 2nd weld1, and so on
			if jointType in jointTypeSpace:
				jointDigit = jointTypeSpace.count(jointType)
				jointName = jointType + str(jointDigit)
			else:
				jointName = jointType
			
			jointTypeSpace.append(jointType)
			
			jointList = [jointName, jointType, jointAxis, jointReference]
			ConnList.append(jointList)
			#getConnection(jointNodeList)
			connRelationNode = jointNodeList[joint].getElementsByTagName('name')[0] #Assume: In XML file, the name tag of joints tells the connection relation to bodies. It contains base and follower name 
			connRelationList = getText(connRelationNode.childNodes).strip('"').split(r'--')
			baseName = connRelationList[0]
			followerName = connRelationList[1]
			#get CS name of each base and follower
			baseNode = jointNodeList[joint].getElementsByTagName('base')[0]
			baseConnNode = baseNode.getElementsByTagName('connection')[0]
			baseCSRefNumAttrNode = baseConnNode.getElementsByTagName('Frame')[0]
			baseCSRefNumAttr = baseCSRefNumAttrNode.attributes["ref"]
			baseCSRefVal = baseCSRefNumAttr.value
			
			followerNode = jointNodeList[joint].getElementsByTagName('follower')[0]
			followerConnNode = followerNode.getElementsByTagName('connection')[0]
			followerCSRefNumAttrNode = followerConnNode.getElementsByTagName('Frame')[0]
			followerCSRefNumAttr = followerCSRefNumAttrNode.attributes["ref"]
			followerCSRefVal = followerCSRefNumAttr.value
			
			baseList = [baseName, baseCSRefVal]
			baseInfoList.append(baseList)
			followerList = [followerName, followerCSRefVal]
			followerInfoList.append(followerList)

		#print 'ConnList', ConnList
		#print 'baseInfoList', baseInfoList
		#print 'followerInfoList', followerInfoList
		#Find the real CS name according to the name of base (or follower) and the ref attribute from connection node

		baseInfoList2 = []
		for baseName in baseInfoList:
			for body in xmlfile.getElementsByTagName('Body'):
				bodynameNode = body.getElementsByTagName('name')[0]
				bodyname = getText(bodynameNode.childNodes).strip('"')
				if bodyname == baseName[0]:
					baseCSRefVal = baseName[1]
					csFrameList = body.getElementsByTagName('Frame')
					for csFrame in csFrameList:
						try:
							frameAttr = csFrame.attributes["ref"]
							
						except:
							continue
						if frameAttr.value == baseCSRefVal:
							csNameNode = csFrame.getElementsByTagName('name')[0]
							baseCSName = getText(csNameNode.childNodes).strip('"')
							baseInfoNew = [bodyname, baseCSName]
							baseInfoList2.append(baseInfoNew)
			if baseName[0] == 'RootGround': #Directly make rootGround CS = none
				baseInfoNew = [baseName[0], 'None']
				baseInfoList2.append(baseInfoNew)
		#print 'baseInfoList2',baseInfoList2


		followerInfoList2 = []
		for followerName in followerInfoList:
			for body in xmlfile.getElementsByTagName('Body'):
				bodynameNode = body.getElementsByTagName('name')[0]
				bodyname = getText(bodynameNode.childNodes).strip('"')
				if bodyname == followerName[0]:
					#print 'followerName[0]',followerName[0]
					followerCSRefVal = followerName[1]
					csFrameList = body.getElementsByTagName('Frame')
					for csFrame in csFrameList:
						try:
							frameAttr = csFrame.attributes["ref"]
							
						except:
							continue
						if frameAttr.value == followerCSRefVal:
							csNameNode = csFrame.getElementsByTagName('name')[0]
							followerCSName = getText(csNameNode.childNodes).strip('"')
							followerInfoNew = [bodyname, followerCSName]
							#print 'followerInfoNew',followerInfoNew
							followerInfoList2.append(followerInfoNew)
							#print 'followerInfoList2',followerInfoList2

		#print 'baseInfoList2', baseInfoList2
		#print 'followerInfoList2', followerInfoList2

		for subJointNum in xrange(len(ConnList)):
			ConnList[subJointNum].append(baseInfoList2[subJointNum])
			ConnList[subJointNum].append(followerInfoList2[subJointNum])
		return ConnList

def main():
	import os
	filePath = os.path.join(os.getcwd(),'fourbar','fourbar.xml')
	xmlparsetest = XMLparsetool(filePath)
	bodyListCG = xmlparsetest.getCSList('CG')
	bodyListCS = xmlparsetest.getCSList('CS')
	connList = xmlparsetest.getConnList()
	
	print 'bodyListCG', bodyListCG
	print 'bodyListCS', bodyListCS
	print 'connList', connList

# test stuff
if __name__ == '__main__':
	main()