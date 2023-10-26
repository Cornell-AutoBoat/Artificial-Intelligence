# Artificial-Intelligence
Code for the ASV's decision-making. Boat brains!

## Changing the code
To make changes to the code, clone the repo by selecting '<> Code' and copying the HTTPS link. Open VSCode, select 'Clone Git Repository...', copy the link, and follow the instructions.

Check out to a new branch to make changes:
```
git checkout <branchname>
```
While working on your branch, changes might be made to the main branch. You can check for changes with ```git status```. To incorporate these changes in your branch, first stage your changes:
```
git add .
git commit -m "some message about your changes"
```
Then checkout to main, pull the changes, check back to your branch, and rebase with main:
```
git checkout main
git pull
git checkout <branchname>
git rebase main
```
You will be asked to reconcile any differences between your branch and the main branch. **Make sure to do this rebasing process before you make any pull requests**.

You can push your changes to your branch after commiting them, and should do this before making a pull request:
```
git push origin <branchname>
```
To make changes to the main branch, you must submit a pull request. There are a few ways to do this.
1) After pushing to your branch, a link will be output in terminal: "Create a pull request for 'branchname' on Github by visiting: ...". You can follow this link in your browser.
2) Or, go to the Github website for the repo. Click on 'Pull requests'. Click 'New pull request' and select your branch. Then click 'Create pull request'.
Please make sure your pull request has a fitting title and add a description describing your changes. Focus on how your changes impact the overall behavior of the code. You must also add two reviewers. At least one of these reviewers must be a lead (FA23: Amelia Kovacs, Eric Zhang, Taha Jafry, Kevin Cui) and the 'point person' of the project (Joanna Lin for AI FA23).

Remember that the changes you make to this repository will not automatically be included in the onboard software. To include them, you must run the submodule update/init commands in the AutoBoat-Onboard-Software repo. See the README of that repo for more detailed instructions.

## Testing the code
To test the code, there is a unit test suite in test.py. This is primarily for testing utils functions (src/control_tasks/utils.py). Run the tests in command line using ```python3 test.py```.

Happy coding!!!
