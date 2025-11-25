#region: File imports
import subprocess
import time
#endregion


def main():
        
        # Check if ACC_Development directory exists
        directoryCreateStatus = subprocess.call("ls /home/$USER/Documents/ACC_Development", shell=True)

        # The directory already exists....
        if directoryCreateStatus ==0:
            # Create a backup copy
            print("Directory already exists....Creating a backup copy ")

            timestr = time.strftime("%Y%m%d-%H%M%S")
            backupDir = "mv /home/$USER/Documents/ACC_Development /home/$USER/Documents/ACC_Development_backup_"+timestr
            directoryCreateStatus = subprocess.call(backupDir, shell=True)
            createNewDir = subprocess.call("mkdir /home/$USER/Documents/ACC_Development", shell=True)
        else :
            directoryCreateStatus = subprocess.call("mkdir /home/$USER/Documents/ACC_Development", shell=True)

        ##############################################################
        ############### CLONE REPOSITORIES ###########################
        ##############################################################

        # Clone Student Competitions repo to get docker container resources and custom Isaac ROS common files 
        cloneStudentCompetitionsRepo = subprocess.call("cd /home/$USER/Documents/ACC_Development/; git clone -b main https://github.com/quanser/student-competitions.git", shell=True)
        if cloneStudentCompetitionsRepo!=0:
            subprocess.call("sudo apt-get install git", shell = True)
            cloneStudentCompetitionsRepo = subprocess.call("cd /home/$USER/Documents/ACC_Development/; git clone https://github.com/quanser/student-competitions.git", shell=True)

        # Clone Quanser Academic Resources repo to get qcar 2 specific resources
        cloneAcademicResources = subprocess.call("cd /home/$USER/Documents/ACC_Development/; git clone -b main https://github.com/quanser/Quanser_Academic_Resources.git", shell=True)
        if cloneAcademicResources!=0:
            subprocess.call("sudo apt-get install git", shell = True)
            cloneStudentCompetitionsRepo = subprocess.call("cd /home/$USER/Documents/ACC_Development/; git clone -b main https://github.com/quanser/Quanser_Academic_Resources.git", shell=True)



        ##############################################################
        ##### TRANSFER CLONED FILES TO ACC_DEVELOPMENT FOLDER ########
        ##############################################################

        # Target folder structure:
        # ACC_Development
        #    ├── backup
        #    ├── Development
        #           ├── ros2
        #           ├── python_dev
        #    ├── docker
        #           ├── development_docker (was isaac_ros)
        #           ├── quanser_docker (was virtual_qcar2)
        #           ├── 0_libraries
        #           ├── miscs
        #    ├── isaac_ros_common
        #

        ### Source is from student-competitions repo ###

        # Make docker folder in ACC DEVELOPMENT
        makeDockerFolder =  subprocess.call(" mkdir /home/$USER/Documents/ACC_Development/docker",shell=True)
        if makeDockerFolder !=0:
            print("Issue creating Docker folder")
            return

        # Copy quanser docker files into Docker folder in ACC DEVELOPMENT
        copyQuanserDockerFiles = subprocess.call("cp -r /home/$USER/Documents/ACC_Development/student-competitions/events/common/Virtual_ROS_Resources/env_setup/docker_resources/quanser_docker /home/$USER/Documents/ACC_Development/docker", shell=True)

        if copyQuanserDockerFiles !=0:
            print("Unable to copy the Quanser Docker Files .... please delete folder ACC_Development folder and try again.. ")
            return

        # Copy development docker files into Docker folder in ACC DEVELOPMENT
        copyDevelopmentDockerFiles = subprocess.call("cp -r /home/$USER/Documents/ACC_Development/student-competitions/events/common/Virtual_ROS_Resources/env_setup/docker_resources/development_docker /home/$USER/Documents/ACC_Development/docker", shell=True)

        if copyDevelopmentDockerFiles !=0:
            print("Unable to copy the Development Docker Files.... please delete folder ACC_Development folder and try again.. ")
            return
        
        # Make miscs folder in docker folder in ACC DEVELOPMENT
        makeMiscsFolder =  subprocess.call(" mkdir /home/$USER/Documents/ACC_Development/docker/miscs",shell=True)
        if makeMiscsFolder !=0:
            print("Issue creating miscs folder")
            return
        
        # Copy misc files into miscs folder in docker folder in ACC DEVELOPMENT
        copyMiscFiles = subprocess.call("cp -r /home/$USER/Documents/ACC_Development/student-competitions/events/common/Virtual_ROS_Resources/env_setup/docker_resources/miscs /home/$USER/Documents/ACC_Development/docker/miscs", shell=True)
        if copyMiscFiles !=0:
            print("Cannot copy misc files... please make sure the student-competitions repo has been cloned correctly...")
            return

        # Make isaac_ros_common folder in ACC DEVELOPMENT
        makeIsaacROSCommonFolder =  subprocess.call(" mkdir /home/$USER/Documents/ACC_Development/isaac_ros_common",shell=True)
        if makeIsaacROSCommonFolder !=0:
            print("Issue creating isaac_ros_common folder")
            return

        # Copy Isaac ROS Common files into isaac_ros_common folder in ACC DEVELOPMENT
        copyIsaacROSCommonFiles = subprocess.call("cp -r /home/$USER/Documents/ACC_Development/student-competitions/events/common/Virtual_ROS_Resources/env_setup/isaac_ros_common /home/$USER/Documents/ACC_Development", shell=True)
        if copyIsaacROSCommonFiles !=0:
            print("Cannot copy Isaac ROS Common files... please make sure the student-competitions repo has been cloned correctly...")
            return

        ### Source is from Academic Resources repo ###
        # make Development folder in ACC DEVELOPMENT
        makeDevelopmentFolder =  subprocess.call(" mkdir /home/$USER/Documents/ACC_Development/Development",shell=True)
        if makeDevelopmentFolder !=0:
            print("Issue creating Development folder")
            return
        
        # Copy ROS Files into Development folder in ACC DEVELOPMENT
        copyQCarROSFiles =  subprocess.call("cp -r /home/$USER/Documents/ACC_Development/Quanser_Academic_Resources/5_research/sdcs/qcar2/ros2 /home/$USER/Documents/ACC_Development/Development",shell=True)
        if copyQCarROSFiles !=0:
            print("Cannot move QCar2 ROS2 files... please make sure the Quanser_Academic_Resources repo has been cloned correctly...")
            return
        
        # copy python dev to Development folder in ACC DEVELOPMENT
        copyPythonDevToROSDocker =  subprocess.call("cp -r /home/$USER/Documents/ACC_Development/Quanser_Academic_Resources/5_research/sdcs /home/$USER/Documents/ACC_Development/Development",shell=True)
        if copyPythonDevToROSDocker !=0:
            print("Cannot move QCar2 python dev files... please make sure the Quanser_Academic_Resources repo has been cloned correctly...")
            return
        
        # rename sdcs folder to python_resources
        renameSDCSFolder = subprocess.call("mv /home/$USER/Documents/ACC_Development/Development/sdcs /home/$USER/Documents/ACC_Development/Development/python_resources", shell=True)
        if renameSDCSFolder !=0:
            print("Could not rename sdcs folder to python_resources")
            return

        # copy the 0_libraries folder into docker folder 
        copyLibrariesToROSDocker =  subprocess.call("cp -r /home/$USER/Documents/ACC_Development/Quanser_Academic_Resources/0_libraries /home/$USER/Documents/ACC_Development/docker",shell=True)
        if copyLibrariesToROSDocker !=0:
            print("Issue copying libraries to docker folder... please make sure the Quanser_Academic_Resources repo has been cloned correctly...")
            return




        ##############################################################
        ########### CLEANUP AND BACKUP OF REPOSITORIES ##############
        ##############################################################

        # create a backup directory
        createBackupDir = subprocess.call("mkdir /home/$USER/Documents/ACC_Development/backup", shell=True)
        if createBackupDir !=0:
            print("Could not create backup directory")
            return
        # copy repos to backup directory
        copyAcademicRepo =  subprocess.call("cp -r /home/$USER/Documents/ACC_Development/Quanser_Academic_Resources /home/$USER/Documents/ACC_Development/backup", shell=True)
        if copyAcademicRepo !=0:
            print("Issue creating backup of academic repo")
            return
        copyStudentCompRepo =  subprocess.call("cp -r /home/$USER/Documents/ACC_Development/student-competitions /home/$USER/Documents/ACC_Development/backup", shell=True)
        if copyStudentCompRepo !=0:
            print("Issue creating backup of academic repo")
            return
        # delete cloned repos
        cleanupAcademicRepo = subprocess.call( "rm -fr /home/$USER/Documents/ACC_Development/Quanser_Academic_Resources", shell=True)
        cleanupSudentCompRepo = subprocess.call( "rm -fr /home/$USER/Documents/ACC_Development/student-competitions", shell=True)
        if cleanupAcademicRepo!=0 or cleanupSudentCompRepo!=0:
            print("Could not delete repos")

        print("System configured correctly! Good Luck!")



# Using the special variable
if __name__=="__main__":
    main()
