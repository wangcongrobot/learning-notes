# Git Usage

fatal: remote origin already exists.

You can change the remote origin in your Git configuration with the following line:

git remote set-url origin git@github.com:username/projectname.git

## Git Large File Storage

https://help.github.com/en/github/managing-large-files/installing-git-large-file-storage

1. go to [here](https://git-lfs.github.com/) and download the [install file](https://github.com/git-lfs/git-lfs/releases/download/v2.9.1/git-lfs-linux-amd64-v2.9.1.tar.gz)
2. go to the folder `git-lfs-linux-xxx`, run `sudo ./install`, and you will get the information: `> Git LFS initialized.`
3. verify that the installation was successful: `git lfs install; Git LFS initialized`