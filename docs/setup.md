---
layout: default
title: Setup
tagline: An invitation to aerospace control
description: How to download and run course code
---

## Contents
{:.no_toc}

* This text will be replaced by a table of contents (excluding the above header) as an unordered list
{:toc}

---

# MacOS

### Learn command-line basics

##### How to open a terminal

When we say "open a terminal," what we mean is to start the **Terminal** application. Here is one way to do that:

* Click the Launchpad icon in the Dock.
* Type "Terminal" in the search field.
* Click Terminal.

See documentation on [Open Terminal](https://support.apple.com/guide/terminal/open-or-quit-terminal-apd5265185d-f365-44cb-8b09-71a064a42125/mac) for more information. Note that it is often helpful to have more than one terminal window open at the same time (or more than one tab in the same window).

##### How to run a command

When we say "run a command," what we mean is to type something into the terminal window and press return. For example, suppose we said:

> run the command `pwd` to find your current working directory

You would type `pwd` into the terminal window and press return, with the result being something like this:

```
(base) timothybretl@Timothys-MacBook-Pro ~ % pwd
/Users/timothybretl
```

See documentation on [Execute commands and run tools in Terminal on Mac for more information](https://support.apple.com/guide/terminal/execute-commands-and-run-tools-apdb66b5242-0d18-49fc-9c47-a2498b7c91d5/mac). Also see the [Command Line Primer](https://developer.apple.com/library/archive/documentation/OpenSource/Conceptual/ShellScripting/CommandLInePrimer/CommandLine.html) for a list of frequently used commands.

##### How to change the working directory

All the files on your computer are organized in folders, which are commonly referred to as "directories." When you are working on the command line in a terminal, you are working in one of these directories. Commands you run can find files in that directory, but cannot (by default) find files in other directories.

When we say "change the working directory," we mean exactly that --- telling the terminal the directory in which you want to work.

To do this, we run the command

```
cd path/to/directory
```

where "`path/to/directory`" is replaced by the location of the directory in which you want to work. One easy way way to find this location (i.e., the "path" to your directory) is by dragging its folder from the Finder into your terminal window (see documentation on [Drag items into a Terminal window on Mac](https://support.apple.com/guide/terminal/drag-items-into-a-terminal-window-trml106/mac)). In particular, I would first type "`cd `" (note the single trailing space):

```
(base) timothybretl@Timothys-MacBook-Pro ~ % cd 
```

Then, I would drag a folder into the terminal window and press return. For instance, suppose I had created a folder called `ae353-sp23` somewhere on my computer and dragged it in, then pressed return --- I would see something like this:

```
(base) timothybretl@Timothys-MacBook-Pro ~ % cd /Users/timothybretl/Documents/ae353-sp23
(base) timothybretl@Timothys-MacBook-Pro ae353-sp23 %
```

See documentation on [Specify files and folders in Terminal on Mac](https://support.apple.com/guide/terminal/specify-files-and-folders-apd3cf6fe02-3ec8-48f1-951f-866e52955fc8/mac) for other ways to specify the path to a directory.


### Do this once

##### Install xcode command line tools

Open a terminal and run this command, accepting all default options:

```
xcode-select --install
```

You may be asked to restart your computer during or after this process. Please do so.


##### Install conda

[Conda](https://docs.conda.io/) is a package manager that you will use to install python, along with a number of required tools.

Open a terminal. Check if conda is already installed by trying to update it to its latest version (something you should do from time to time anyway):

```
conda update -n base conda
```

If this process succeeds, then conda is installed, and you should skip to the next step. If this process does not succeed, then you still need to install conda, and you should continue with this step.

Follow the instructions for [Installing on macOS](https://docs.conda.io/projects/conda/en/latest/user-guide/install/macos.html). It is easiest to use the [Miniconda installer](https://docs.conda.io/en/latest/miniconda.html), in particular the appropriate `.pkg` file for your architecture (Intel or Silicon) --- just double-click the `.pkg` file after it downloads and accept all the default options (you need not "verify your installer hashes" or run a bash command or anything).

##### Create a conda environment

An "environment" is like a sandbox where you can install software without causing any conflict with other things you might have installed on your computer. To create an environment for work in AE353, open a terminal and run this command:

```
conda create -n ae353
```

You may see something like `WARNING: A conda environment already exists` and be asked if you want to `Remove existing environment`. This means that you already created an environment called `ae353`. Perhaps something went wrong and you are creating it again --- in this case, type `y` to remove the existing environment and proceed to recreate it. From then on, accept all default options.

Finally, run the following commands (copy *the entire thing*, paste it into your terminal, and press return):

```
conda activate ae353
conda config --env --add channels conda-forge
conda config --env --set channel_priority strict
conda install -y python=3 numpy scipy sympy matplotlib \
    notebook ipywidgets imageio imageio-ffmpeg \
    pybullet meshcat-python
```

##### Get the code

Open a terminal and change the working directory to a folder in which you would like to put all your files. Then, use [git](https://git-scm.com/) to download the code from our [ae353 github repository]({{ site.github.repository_url }}) by running this command:

```
git clone https://github.com/tbretl/ae353-sp23.git
```

This process will take very little time. When it completes, you should find a new folder called `ae353-sp23` inside your working directory.

### Do this every time

##### Change your working directory

Open a terminal and change your working directory to `ae353-sp23`, wherever you put this.

##### Get the latest version of the code

Run this command:

```
git pull
```

Do not worry, this will not overwrite any of your own work. If you see any errors or warnings, post a note to [Campuswire](https://campuswire.com/c/G9558828D/) and course staff will help resolve them.

##### Activate your conda environment

Run this command:

```
conda activate ae353
```

You should see the prefix to your terminal prompt change from `(base)` to `(ae353)`. This means you are in the conda environment you created for work with AE353.

##### Start a jupyter notebook

Run this command:

```
jupyter notebook
```

A browser window should open with the jupyter notebook interface. You can now navigate to and open any of the notebooks (with extension `.ipynb`) used for in-class examples or for design projects.

**We strongly recommend you duplicate and work with a copy of any given notebook rather than working with the original.** Feel free to ignore this suggestion if you are a `git` expert.


# Windows

### Learn command-line basics

##### How to open an anaconda powershell

When we say "open an anaconda powershell," what we mean is to start the **Anaconda Powershell Prompt (Miniconda)** application. Here is one way to do that:

* Click the Windows search bar near the bottom-left corner of your desktop.
* Type "anaconda" into the search field.
* Click **Anaconda Powershell Prompt (Miniconda)**.

Note that this application will only exist after you follow the instructions to [Install conda](#install-conda-1).

##### How to run a command

When we say "run a command," what we mean is to type something into the window of an anaconda powershell and press enter. See [this beginners guide](https://www.makeuseof.com/tag/a-beginners-guide-to-the-windows-command-line/) to [Windows Commands](https://docs.microsoft.com/en-us/windows-server/administration/windows-commands/windows-commands).

##### How to change the working directory

All the files on your computer are organized in folders, which are commonly referred to as "directories." When you are working on the command line, you are working in one of these directories. Commands you run can find files in that directory, but cannot (by default) find files in other directories.

When we say "change the working directory," we mean exactly that --- telling an anaconda powershell the directory in which you want to work.

To do this, we run the command

```
cd path\to\directory
```

where "`path\to\directory`" is replaced by the location of the directory in which you want to work. One easy way way to find this location (i.e., the "path" to your directory) is by dragging its folder from the File Explorer into your powershell window (see documentation on [Quickly Copy Files Paths to Your Command Prompt via Drag and Drop](https://lifehacker.com/quickly-copy-file-paths-to-your-command-prompt-via-drag-5382503). In particular, I would first type "`cd `" (note the single trailing space):

```
C:\Users\jakek>cd 
```

Then, I would drag a folder into the powershell window and press enter. For instance, suppose I had created a folder called `ae353-sp23` somewhere on my computer and dragged it in, then pressed enter --- I would see something like this:

```
C:\Users\jakek>cd C:\Users\jakek\OneDrive\Documents\ae353-sp23
C:\Users\jakek\OneDrive\Documents\ae353-sp23>
```

See documentation on [Find and Open Files using Windows Command Prompt](https://www.faqforge.com/windows/windows-10/find-and-open-files-using-windows-command-prompt/) for a way to search for the directory location of files on your computer.


### Do this once

##### Install conda

[Conda](https://docs.conda.io/) is a package manager that you will use to install python, along with a number of required tools.

Try to open an **anaconda powershell**.

If you can, then conda may already be installed. Try to update it to its latest version (something you should do from time to time anyway) by running the following command in the powershell:

```
conda update -n base conda
```

If this process succeeds, then conda is installed properly, and you should skip to the next step. If this process does not succeed, or if you couldn't open an anaconda powershell in the first place, then you still need to install conda, and you should continue with this step.

Follow the instructions for [Installing on Windows](https://docs.conda.io/projects/conda/en/latest/user-guide/install/windows.html). It is easiest to use the [Miniconda installer](https://conda.io/miniconda.html), in particular the [Miniconda3 Windows 64-bit](https://repo.anaconda.com/miniconda/Miniconda3-latest-Windows-x86_64.exe) --- just double-click the `.exe` file after it downloads and accept all the default options (you need not "verify your installer hashes").

##### Create a conda environment

An "environment" is like a sandbox where you can install software without causing any conflict with other things you might have installed on your computer. To create an environment for work in AE353, open an **anaconda powershell** and run this command:

```
conda create -n ae353
```

You may see something like `WARNING: A conda environment already exists` and be asked if you want to `Remove existing environment`. This means that you already created an environment called `ae353`. Perhaps something went wrong and you are creating it again --- in this case, type `y` to remove the existing environment and proceed to recreate it. From then on, accept all default options.

Finally, run the following commands (copy *the entire thing*, paste it into your powershell, and press enter):

```
conda activate ae353
conda config --env --add channels conda-forge
conda config --env --set channel_priority strict
conda install -y python=3 numpy scipy sympy matplotlib
conda install -y notebook ipywidgets imageio imageio-ffmpeg
conda install -y pybullet meshcat-python
conda install -y git
```

Note that (unlike for MacOS) we are using conda to install [git](https://git-scm.com/) --- this seemed simplest, rather than figuring out how to make system-installed software accessible to the anaconda powershell. The only consequence of this choice is that you will need to be in the `ae353` environment to run git.

##### Get the code

Open an **anaconda powershell** and change the working directory to a folder in which you would like to put all your files.

Then, activate your conda environment:

```
conda activate ae353
```

Finally, use [git](https://git-scm.com/) to download the code from our [ae353 github repository]({{ site.github.repository_url }}) by running this command:

```
git clone https://github.com/tbretl/ae353-sp23.git
```

This process will take very little time. When it completes, you should find a new folder called `ae353-sp23` inside your working directory.

### Do this every time

##### Change your working directory

Open an **anaconda powershell** and change your working directory to `ae353-sp23`, wherever you put this.

##### Activate your conda environment

Run this command:

```
conda activate ae353
```

You should see the prefix to your powershell prompt change from `(base)` to `(ae353)`. This means you are in the conda environment you created for work with AE353.

##### Get the latest version of the code

Run this command:

```
git pull
```

Do not worry, this will not overwrite any of your own work. If you see any errors or warnings, post a note to [Campuswire](https://campuswire.com/c/G9558828D/) and course staff will help resolve them.

##### Start a jupyter notebook

Run this command:

```
jupyter notebook
```

A browser window should open with the jupyter notebook interface. You can now navigate to and open any of the notebooks (with extension `.ipynb`) used for in-class examples or for design projects.

**We strongly recommend you duplicate and work with a copy of any given notebook rather than working with the original.** Feel free to ignore this suggestion if you are a `git` expert.


# Linux

Post a note to [Campuswire](https://campuswire.com/c/G9558828D/) and we will help you.



