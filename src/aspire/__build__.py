########## INIT ####################################################################################
import subprocess, os, sys



########## FAST DOWNWARD ###########################################################################
def build_FD():
    """  """

    fChunk   = str( __file__ ).split('/')[-1]
    modPath  = str( __file__ )[:-len(fChunk)]
    buildLoc = os.path.join( modPath, "pddlstream/downward/build.py" )
    print( f"\nAbout to run {buildLoc} ...\n(This can take several minutes. Please be patient.)\n" )
    process = subprocess.Popen( f"python3.10 {buildLoc}", 
                                shell  = True,
                                stdout = sys.stdout, 
                                stderr = sys.stderr )

    # wait for the process to terminate
    errcode = process.wait()
    print( f"\nInstalled FastDownward!\nExited with code: {errcode}\n" )

