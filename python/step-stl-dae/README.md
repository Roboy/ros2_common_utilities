.step-to-.dae
=============

Script that helps to convert 3D models in .step format to .dae format using FreeCAD and Meshlab


Dependencies
------------

* [FreeCAD](http://sourceforge.net/apps/mediawiki/free-cad/index.php?title=Main_Page)
* [MeshLab](http://meshlab.sourceforge.net/)
* Python v2.7


Usage
------------

To convert all step files in current directory use

$ step-to-dae 

To convert a single step file (foo.step) use

$ step-to-dae foo.step

All processed step files are stored in a folder called "done_step" nested in the current directory


Known Bugs
------------

The script is not able to process ~30% of tested step files. The program fails during the conversion of the step file into a binary stl with FreeCAD. In some cases it hangs and in some others it breaks while using nearly all RAM.

SOME of this step files have a large amount of shapes and manually removing some of these shapes with FreeCAD makes the file convertible.


Contact
------------

[fede.barabas@gmail.com](mailto:fede.barabas@gmail.com)
