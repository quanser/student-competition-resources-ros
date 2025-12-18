#region: File imports
import subprocess
import time
#endregion

# if returns 1, then a cleanup is required
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

        # Clone student competition resources ros repo to get docker container resources and custom Isaac ROS common files
        cloneStudentCompetitionResourcesROS = subprocess.call("cd /home/$USER/Documents/ACC_Development/; git clone -b main https://github.com/quanser/student-competition-resources-ros.git", shell=True)
        if cloneStudentCompetitionResourcesROS!=0:
            subprocess.call("sudo apt-get install git", shell = True)
            cloneStudentCompetitionResourcesROS = subprocess.call("cd /home/$USER/Documents/ACC_Development/; git clone -b main https://github.com/quanser/student-competition-resources-ros.git", shell=True)

        # Clone Quanser Academic Resources repo to get qcar 2 specific resources
        cloneAcademicResources = subprocess.call("cd /home/$USER/Documents/ACC_Development/; git clone -b dev-qcar https://github.com/quanser/Quanser_Academic_Resources.git", shell=True)
        if cloneAcademicResources!=0:
            subprocess.call("sudo apt-get install git", shell = True)
            cloneStudentCompetitionsRepo = subprocess.call("cd /home/$USER/Documents/ACC_Development/; git clone -b dev-qcar https://github.com/quanser/Quanser_Academic_Resources.git", shell=True)



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
        #    ├── isaac_ros_common
        #

        ### Source is from student-competition-resources-ros repo ###

        # Make docker folder in ACC DEVELOPMENT
        makeDockerFolder =  subprocess.call(" mkdir /home/$USER/Documents/ACC_Development/docker",shell=True)
        if makeDockerFolder !=0:
            print("Issue creating Docker folder")
            return 1

        # Copy quanser docker files into Docker folder in ACC DEVELOPMENT
        copyQuanserDockerFiles = subprocess.call("cp -r /home/$USER/Documents/ACC_Development/student-competition-resources-ros/Virtual_ROS_Resources/env_setup/docker_resources/quanser_docker /home/$USER/Documents/ACC_Development/docker", shell=True)
        if copyQuanserDockerFiles !=0:
            print("Unable to copy the Quanser Docker Files .... please delete folder ACC_Development folder and try again.. ")
            return 1

        # Copy development docker files into Docker folder in ACC DEVELOPMENT
        copyDevelopmentDockerFiles = subprocess.call("cp -r /home/$USER/Documents/ACC_Development/student-competition-resources-ros/Virtual_ROS_Resources/env_setup/docker_resources/development_docker /home/$USER/Documents/ACC_Development/docker", shell=True)
        if copyDevelopmentDockerFiles !=0:
            print("Unable to copy the Development Docker Files.... please delete folder ACC_Development folder and try again.. ")
            return 1

        # rename .isaac_ros_common-config-quanser file as .isaac_ros_common-config file
        renameIsaacrosconfig = subprocess.call("mv /home/$USER/Documents/ACC_Development/docker/development_docker/isaac_ros_common_config/.isaac_ros_common-config-quanser /home/$USER/Documents/ACC_Development/docker/development_docker/isaac_ros_common_config/.isaac_ros_common-config", shell=True)
        if renameIsaacrosconfig !=0:
            print("Could not rename .isaac_ros_common-config-quanser to .isaac_ros_common-config")
            return 1

        # Make isaac_ros_common folder in ACC DEVELOPMENT
        makeIsaacROSCommonFolder =  subprocess.call(" mkdir /home/$USER/Documents/ACC_Development/isaac_ros_common",shell=True)
        if makeIsaacROSCommonFolder !=0:
            print("Issue creating isaac_ros_common folder")
            return 1

        # Copy Isaac ROS Common files into isaac_ros_common folder in ACC DEVELOPMENT
        copyIsaacROSCommonFiles = subprocess.call("cp -r /home/$USER/Documents/ACC_Development/student-competition-resources-ros/Virtual_ROS_Resources/env_setup/isaac_ros_common /home/$USER/Documents/ACC_Development", shell=True)
        if copyIsaacROSCommonFiles !=0:
            print("Cannot copy Isaac ROS Common files... please make sure the student-competition-resources-ros repo has been cloned correctly...")
            return 1

        # Copy .isaac_ros_common_config file into isaac_ros_common/scripts folder
        copyIsaacrosconfig = subprocess.call("cp -r /home/$USER/Documents/ACC_Development/docker/development_docker/isaac_ros_common_config/.isaac_ros_common-config /home/$USER/Documents/ACC_Development/isaac_ros_common/scripts", shell=True)
        if copyIsaacrosconfig !=0:
            print("Cannot copy .isaac_ros_common_config... please make sure the student-competition-resources-ros repo has been cloned correctly...")
            return 1

        ### Source is from Academic Resources repo ###
        # make Development folder in ACC DEVELOPMENT
        makeDevelopmentFolder =  subprocess.call(" mkdir /home/$USER/Documents/ACC_Development/Development",shell=True)
        if makeDevelopmentFolder !=0:
            print("Issue creating Development folder")
            return 1
        
        # Copy ROS Files into Development folder in ACC DEVELOPMENT
        copyQCarROSFiles =  subprocess.call("cp -r /home/$USER/Documents/ACC_Development/Quanser_Academic_Resources/5_research/sdcs/qcar2/ros2 /home/$USER/Documents/ACC_Development/Development",shell=True)
        if copyQCarROSFiles !=0:
            print("Cannot move QCar2 ROS2 files... please make sure the Quanser_Academic_Resources repo has been cloned correctly...")
            return 1
        
        # copy python dev to Development folder in ACC DEVELOPMENT
        copyPythonDevToROSDocker =  subprocess.call("cp -r /home/$USER/Documents/ACC_Development/Quanser_Academic_Resources/5_research/sdcs /home/$USER/Documents/ACC_Development/Development",shell=True)
        if copyPythonDevToROSDocker !=0:
            print("Cannot move QCar2 python dev files... please make sure the Quanser_Academic_Resources repo has been cloned correctly...")
            return 1
        
        # rename sdcs folder to python_resources
        renameSDCSFolder = subprocess.call("mv /home/$USER/Documents/ACC_Development/Development/sdcs /home/$USER/Documents/ACC_Development/Development/python_resources", shell=True)
        if renameSDCSFolder !=0:
            print("Could not rename sdcs folder to python_resources")
            return 1
        
        # delete ros files
        rosFlag = 0
        deleteROSFiles = subprocess.call("rm -r /home/$USER/Documents/ACC_Development/Development/python_resources/qcar/hardware/ros1_cpp", shell=True)
        rosFlag += deleteROSFiles
        deleteROSFiles = subprocess.call("rm -r /home/$USER/Documents/ACC_Development/Development/python_resources/qcar/hardware/ros1_python", shell=True)
        rosFlag += deleteROSFiles
        deleteROSFiles = subprocess.call("rm -r /home/$USER/Documents/ACC_Development/Development/python_resources/qcar/hardware/ros2", shell=True)
        rosFlag += deleteROSFiles
        deleteROSFiles = subprocess.call("rm -r /home/$USER/Documents/ACC_Development/Development/python_resources/qcar2/ros2", shell=True)
        rosFlag += deleteROSFiles
        if deleteROSFiles !=0:
            print("There may be issues removing the ROS resources")
            return 1


        # copy the 0_libraries folder into docker folder 
        copyLibrariesToROSDocker =  subprocess.call("cp -r /home/$USER/Documents/ACC_Development/Quanser_Academic_Resources/0_libraries /home/$USER/Documents/ACC_Development/docker",shell=True)
        if copyLibrariesToROSDocker !=0:
            print("Issue copying libraries to docker folder... please make sure the Quanser_Academic_Resources repo has been cloned correctly...")
            return 1
        
        copyLibrariesToROSDockerfiles = subprocess.call("cp -r /home/$USER/Documents/ACC_Development/Quanser_Academic_Resources/0_libraries /home/$USER/Documents/ACC_Development/docker/development_docker/quanser_dev_docker_files",shell=True)
        if copyLibrariesToROSDockerfiles !=0:
            print("Issue copying libraries to docker files folder... please make sure the Quanser_Academic_Resources repo has been cloned correctly...")
            return 1

        ##############################################################
        ########### CLEANUP AND BACKUP OF REPOSITORIES ##############
        ##############################################################

        # create a backup directory
        createBackupDir = subprocess.call("mkdir /home/$USER/Documents/ACC_Development/backup", shell=True)
        if createBackupDir !=0:
            print("Could not create backup directory")
            return 1
        # copy repos to backup directory
        copyAcademicRepo =  subprocess.call("cp -r /home/$USER/Documents/ACC_Development/Quanser_Academic_Resources /home/$USER/Documents/ACC_Development/backup", shell=True)
        if copyAcademicRepo !=0:
            print("Issue creating backup of academic repo")
            return 1
        copyStudentCompRepo =  subprocess.call("cp -r /home/$USER/Documents/ACC_Development/student-competition-resources-ros /home/$USER/Documents/ACC_Development/backup", shell=True)
        if copyStudentCompRepo !=0:
            print("Issue creating backup of academic repo")
            return 1

        # delete cloned repos
        cleanupAcademicRepo = subprocess.call( "rm -fr /home/$USER/Documents/ACC_Development/Quanser_Academic_Resources", shell=True)
        cleanupSudentCompRepo = subprocess.call( "rm -fr /home/$USER/Documents/ACC_Development/student-competition-resources-ros", shell=True)
        if cleanupAcademicRepo!=0 or cleanupSudentCompRepo!=0:
            print("Could not delete repos")
            return 1

        print("System configured correctly! Good Luck!")



# Using the special variable
if __name__=="__main__":
    # enable cleanup of the resources if a failed setup occurs
    cleanupEnable = 1

    cleanupFlag = main()
    # cleanup check
    if cleanupFlag == 1 and cleanupEnable == 1:

        print("Cleaning up the ACC_Development folder due to setup issues...")

        # Remove ACC_Development folder
        removeACCDevelopment =  subprocess.call("rm -r -f /home/$USER/Documents/ACC_Development", shell=True)
        if removeACCDevelopment !=0:
            print("issue removing the ACC_Development folder")

