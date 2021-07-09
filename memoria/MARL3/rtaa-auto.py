#!/usr/bin/python
"""
    PROJECT - SOCS2011

    DESCRIPTION Automation of results

    @copyright: 2011 by cristhian <caguilerac@udec.cl>
    @license: GNU GPL, see COPYING for details.
"""



import os,sys,commands
from collections import defaultdict
import csv

resFileName=""
mapFolder=""
resultsDict=defaultdict(list)

def InitResults():
    """ Function doc

    @return RETURN: nothing
    """
    #Create Dictionary
    resultsDict[0].append('MAP')
    resultsDict[0].append('LookAhead')
    resultsDict[0].append('solution_cost')
    resultsDict[0].append('robotmoves_total1')
    resultsDict[0].append('searches_astar1')
    resultsDict[0].append('runtime')
    resultsDict[0].append('average_expansion_persearch')
    resultsDict[0].append('statexpanded_prune')
    resultsDict[0].append('RUN1')
    resultsDict[0].append('expansions')
    resultsDict[0].append('percolations')
    
 
        
def CloseResults():
    """ Function doc

    @return RETURN: nothing
    """
    f = open(resFileName, 'wt')
    writer = csv.writer(f)
    end=len(resultsDict)
    for i in range(0,end):
        writer.writerow(resultsDict[i])
            
def StoreResults(name):
    """ Function doc

    @param results: Results of the algorithm 
    """
    try:
        file = open('Output-rtaa', 'r')
        old=file.read()
        oldArray=old.split('\n')
        for i in range(0,len(oldArray)-1):
            new=oldArray[i].split(' ')
            indexDICT=len(resultsDict)
            resultsDict[indexDICT].append(name)
            resultsDict[indexDICT].append(new[0])
            resultsDict[indexDICT].append(new[1])
            resultsDict[indexDICT].append(new[2])
            resultsDict[indexDICT].append(new[3])
            resultsDict[indexDICT].append(new[4])
            resultsDict[indexDICT].append(new[5])
            resultsDict[indexDICT].append(new[6])
            resultsDict[indexDICT].append(new[7])
            resultsDict[indexDICT].append(new[8])
            resultsDict[indexDICT].append(new[9])
            resultsDict[indexDICT].append(new[10])
    except IOError:
        print "No se pudo procesar el mapa "+name                
def CompileAndRun():
    """ Function doc

    @param None: 
    @return RETURN: nothing
    """
    res = commands.getoutput("rm *.o Output-rtaa")
    res = commands.getoutput("make")
    res = commands.getoutput("./testall")


def NewMaps():
    print 'START FIX MAPS in '+mapFolder
    dirList=os.listdir(mapFolder)
    old=""
    for fname in dirList:
        if fname.find('.map') != -1:
            if fname.find('.map2') == -1:
                print 'MAP '+fname+'....'
                file = open(mapFolder+fname, 'r')
                for i in file:
                    if i.find('type')!=-1:
                        pass
                    elif i.find('height')!=-1:
                        pass
                    elif i.find('width')!=-1:
                        pass
                    elif i.find('map')!=-1:
                        pass
                    else:
                        old= old +i             
                file.close()
                new=open(mapFolder+fname+'2', 'w')
                new.write(old)
                new.close()
        old=""
    print 'MAPS FIX DONE'
    
    
    
def ChooseMazeType(maze):
    """ Function doc """
    file = open('include.h', 'r')
    old=file.read()
    if algorithm == 'RANDOMMAZE':
        old=old.replace('//#define RANDOMMAZE','#define RANDOMMAZE')
        file.close()
        new=open('include.h', 'w')
        new.write(old)
        new.close()
        print 'RANDOMMAZE CONF DONE'

def  ChooseWidthHeight(Width,Height):
    """ Function doc """
    file = open('include.h', 'r')
    old=""
    for i in file:
        if i.find('#define MAZEWIDTH')!=-1:
            old=old+'#define MAZEWIDTH '+Width +'\n'
        elif i.find('#define MAZEHEIGHT') !=-1:
            old=old+'#define MAZEHEIGHT '+Height + '\n'
        else:
            old=old+i
    file.close()
    new=open('include.h', 'w')
    new.write(old)
    new.close()
    print 'WIDTH AND HEIGHT CONF DONE'

def  ReadMapAndConf(mapfile):
    """ Function doc """
    file = open(mapfile, 'r')
    height = ""
    width=""
    for i in file:
        if i.find('height')!=-1:
            old=i
            old=old.replace('height','')
            old=old.replace('\n','')
            old=old.replace('\t','')
            height=old
        elif i.find('width')!=-1:
            old=i
            old=old.replace('width','')
            old=old.replace('\n','')
            old=old.replace('\t','')
            width=old            
    file.close()
    ChooseWidthHeight(width,height)

def ChooseMAP(mapfile):
    old=""
    file = open('testall.c', 'r')
    for i in file:
        if i.find('read_gamemap("')!=-1:
            old= old + 'read_gamemap("'+mapfile+'");\n'
        else:
            old= old +i             
    file.close()
    new=open('testall.c', 'w')
    new.write(old)
    new.close()
    print 'MAP CONF DONE' 
     
def RunFolder():
    """ Function doc
    @return RETURN: None
    """
    NewMaps() 
    dirList=os.listdir(mapFolder)
    old=""
    i=1
    InitResults()
    for fname in dirList:
        if fname.find('.map') != -1:
            if fname.find('.map2') == -1:
                print str(i)+'.-MAP '+fname +" START"
                ReadMapAndConf(mapFolder+fname)  
                ChooseMAP(mapFolder+fname+'2') 
                CompileAndRun()  
                StoreResults(fname)
                print str(i)+'.-MAP '+fname +" Done"
                i=i+1

    
def ChooseRuns(runs):
    """ Function doc

    @param runs: number of runs
    @return RETURN: None
    """
    old=""
    file = open('include.h', 'r')
    for i in file:
        if i.find('#define RUNS ')!=-1:
            old= old + '\n#define RUNS '+runs+'\n'
        else:
            old= old +i             
    file.close()
    new=open('include.h', 'w')
    new.write(old)
    new.close()
    print 'RUNS CONF DONE'
    


def ChooseLookAhead (ini,step,final):
    """ Function doc
    @param ini: initial lookahead
    @param step: step lookahead
    @param final: final lookahead
    @return RETURN: none
    """
    old=""
    file = open('rtaastar.c', 'r')
    for i in file:
        if i.find('for (lookahead')!=-1:
            old= old + 'for (lookahead = '+ini+'; lookahead <'+final+'; lookahead = lookahead *'+step+')\n'
        else:
            old= old +i             
    file.close()
    new=open('rtaastar.c', 'w')
    new.write(old)
    new.close()
    print 'LookAhead CONF DONE' 
    
           
        
if __name__ == '__main__':
    if len(sys.argv) < 2: # There is argumens
        print 'Wrong number of arguments'
        print 'Example:'
        print './rtaa-auto.py map/randommaze RUNS InitialLookAhead STEPLookAhead ENDLookAhead FOLDER/FILE Folder/file'
        print './rtaa-auto.py map 100 10 4 200 folder ../Maps/Dragon_Age_Origins/'
      
    else:
        resFileName='rtaa_run.csv'
        if sys.argv[1] == 'map':
            ChooseRuns(sys.argv[2]) # RUN
            ChooseLookAhead(sys.argv[3],sys.argv[4],sys.argv[5])
            if sys.argv[6]=='folder':
                mapFolder=sys.argv[7]
                RunFolder()
                CloseResults()
        
        
    
