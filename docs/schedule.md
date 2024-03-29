---
layout: default
title: Schedule
tagline: An invitation to aerospace control
description: What we will do from day to day
---

## Contents
{:.no_toc}

* This text will be replaced by a table of contents (excluding the above header) as an unordered list
{:toc}

---

# Things to do after each class

## Part 1 - Linearization and stability

#### Wednesday, January 18 (Introduction)

* Read the [syllabus](index).
* Follow the ["Setup" instructions](setup) so you can run course code on your own computer.


#### Friday, January 20 (Start talking about PID controllers)

* Fill out [this form to express group preferences for your first design project](https://forms.illinois.edu/sec/673116346).
* Read [the reference page on PID](reference#pid).
* (Optional) Read [Chapter 1.6 of the reference text (Feedback Systems)](http://www.cds.caltech.edu/~murray/books/AM08/pdf/fbs-intro_24Jul2020.pdf), or the whole of Chapter 1, for more information on PID controllers and on the idea of "control systems" in general.
* Complete [HW1 in PrairieLearn](https://us.prairielearn.com/pl/course_instance/130996/assessment/2327509).


#### Monday, January 23 (Finish talking about PID controllers + Simulation)

* Complete the ["Setup" instructions](setup) if you have not already done so, and ask for help (e.g., [campuswire](https://campuswire.com/c/G9558828D/), [office hours](index#instructor), etc.) if you get stuck.
* Complete [HW2 in PrairieLearn](https://us.prairielearn.com/pl/course_instance/130996/assessment/2327697).


#### Wednesday, January 25 (Simulation + Moving from PID to linear state feedback)

* Contact [your partner for the first design project](https://campuswire.com/c/G9558828D/feed/33), arrange a time to meet, and make sure both of you have completed the ["Setup" instructions](setup) if you have not already done so.
* Complete [HW3 in PrairieLearn](https://us.prairielearn.com/pl/course_instance/130996/assessment/2327910).


#### Friday, January 27 (Implementation of linear state feedback)

* Read the [guidelines for the first design project](https://tbretl.github.io/ae353-sp23/projects#design-project-1-cmg).
* Complete [HW4 in PrairieLearn](https://us.prairielearn.com/pl/course_instance/130996/assessment/2327057).


#### Monday, January 30 (Linearization to get a state space model)

* Read the [notes on linearization from class](notes/20230130-linearization.pdf).
* Read the [reference page on state space models](reference#state-space-models), which goes step-by-step through the process of linearization.
* (Optional) Read [Chapter 6.4 of the reference text (Feedback Systems)](http://www.cds.caltech.edu/~murray/books/AM08/pdf/fbs-linsys_24Jul2020.pdf) for more information on linearization, including alternatives to "linearization about an equilibrium point."
* Complete [HW5 in PrairieLearn](https://us.prairielearn.com/pl/course_instance/130996/assessment/2327058).

#### Wednesday, February 1 (Matrix exponential and asymptotic stability)

* Read the [notes on solving closed-loop systems with the matrix exponential function from class](notes/20230201-matrix-exponential.pdf).
* Read the reference page on [the matrix exponential function](reference#the-matrix-exponential-function) and on [asymptotic stability](reference#asymptotic-stability).
* Complete [HW6 in PrairieLearn](https://us.prairielearn.com/pl/course_instance/130996/assessment/2327059).

#### Friday, February 3 (Availability and use of example reports, summary of design process so far)

* Read the [campuswire post about prior work to learn and build upon](https://campuswire.com/c/G9558828D/feed/70) for your projects.
* Submit [DP1 Draft 1](https://tbretl.github.io/ae353-sp23/projects#draft-report-with-theory-by-1159pm-on-friday-february-3) by midnight today.
* Read the [notes on the complete design process so far, from class](notes/20230203-design-process.pdf).
* Complete [HW7 in PrairieLearn](https://us.prairielearn.com/pl/course_instance/130996/assessment/2327060).

#### Monday, February 6 (Diagonalization, Part 1)

* Re-read the reference page on [the matrix exponential function](reference#the-matrix-exponential-function) and on [asymptotic stability](reference#asymptotic-stability).
* Read the [notes on diagonalization (part 1) from class](notes/20230206-diagonalization-part-1.pdf).
* Complete [HW8 in PrairieLearn](https://us.prairielearn.com/pl/course_instance/130996/assessment/2327061).

#### Wednesday, February 8 (Diagonalization, Part 2)

* Read the [notes on diagonalization (part 2) from class](notes/20230208-diagonalization-part-2.pdf).
* Complete [HW9 in PrairieLearn](https://us.prairielearn.com/pl/course_instance/130996/assessment/2330822).

#### Friday, February 10 (Exams, Projects)

* Submit [DP1 Draft 2](https://tbretl.github.io/ae353-sp23/projects#draft-report-with-results-by-1159pm-on-friday-february-10) by midnight today.
* Sign up for and take Exam 1 in the CBTF

#### Monday, February 13 (No class)

* Good luck on your exam! I will hold an extra office hour during our normal class time.

## Part 2 - Controller design

#### Wednesday, February 15 (Eigenvalue placement)

* Read the [notes on eigenvalue placement from class](notes/20230215-placement.pdf).
* Complete [HW10 in PrairieLearn](https://us.prairielearn.com/pl/course_instance/130996/assessment/2327062).
* Fill out [this form to express group preferences for your second design project](https://forms.illinois.edu/sec/200984506).

#### Friday, February 17 (Second design project)

* Submit all [final deliverables for the first design project](projects#design-project-1-cmg).
* Read the [guidelines for the second design project](projects#design-project-2-differential-drive-robot-in-artificial-gravity)
* Contact [your partner for the second design project](https://campuswire.com/c/G9558828D/feed/148) and make a plan to get started.


#### Monday, February 20 (Ackermann's method - theory)

* Read the [notes on Ackermann's method (theory) from class](notes/20230220-acker.pdf).
* Complete [HW11 in PrairieLearn](https://us.prairielearn.com/pl/course_instance/130996/assessment/2327063).

#### Wednesday, February 22 (Ackermann's method - implementation + Controllability)

* Read the [notes on Ackermann's method (implementation) and on controllability from class](notes/20230222-controllability.pdf).
* Read the [reference page on eigenvalue placement](reference#eigenvalue-placement-by-controllable-canonical-form)
* (Optional) Watch supplementary videos on Ackermann's method:
    * [Ackermann's Method, Part 1 (Eigenvalues are invariant to coordinate transformation)](https://mediaspace.illinois.edu/media/t/1_93vewoav/)
    * [Ackermann's Method, Part 2 (Controllable canonical form)](https://mediaspace.illinois.edu/media/t/1_rbf0x31w/)
    * [Ackermann's Method, Part 3 (How to put a system in controllable canonical form)](https://mediaspace.illinois.edu/media/t/1_e6r6ljxc/)
    * [Ackermann's Method, Part 4 (Putting it all together)](https://mediaspace.illinois.edu/media/t/1_sf1ydkq4/)
* Complete [HW12 in PrairieLearn](https://us.prairielearn.com/pl/course_instance/130996/assessment/2327064).

#### Wednesday, February 24 (LQR introduction)

* Read the [notes on LQR (introduction) from class](notes/20230224-lqr-intro.pdf).
* Complete [HW13 in PrarieLearn](https://us.prairielearn.com/pl/course_instance/130996/assessment/2327065).

#### Monday, February 27 (LQR problem statement)

* Read the [notes on LQR (problem statement) from class](notes/20230227-lqr-problem.pdf).
* Complete [HW14 in PrarieLearn](https://us.prairielearn.com/pl/course_instance/130996/assessment/2327066).

#### Wednesday, March 1 (LQR details)

* Read the [notes on LQR (details) from class](notes/20230301-lqr-details.pdf).

#### Friday, March 3 (LQR examples)

* Submit [DP2 Draft 2](projects#draft-report-with-results-by-1159pm-on-friday-march-3) by midnight today.
* Sign up for and take Exam 2 in the CBTF

#### Monday, March 6 (No class)

* Good luck on your exam! I will hold an extra office hour during our normal class time.

## Part 3 - Observer design

#### Wednesday, March 8 (Observer implementation)

* Complete [HW15 in PrairieLearn](https://us.prairielearn.com/pl/course_instance/130996/assessment/2327070).

#### Friday, March 10 (Observer design and analysis)

* Fill out [this form to express group preferences for your third design project](https://forms.illinois.edu/sec/1605289650).
* Submit all [final deliverables for the second design project](projects#segbot-your-deliverables).
* Read the [notes on observers (part 1) from class](notes/20230310-observers-part-1.pdf).
* Complete [HW16 in PrairieLearn](https://us.prairielearn.com/pl/course_instance/130996/assessment/2327067).
* (Optional) Watch supplementary videos on observer design and analysis:
    * [What is an observer?](https://mediaspace.illinois.edu/media/t/1_bwsv03zv/292883062)
    * [Do observers make sense?](https://mediaspace.illinois.edu/media/t/1_xf9cytda/292883062)
    * [When does an observer work?](https://mediaspace.illinois.edu/media/t/1_ma7mf2v3/292883062)
    * [How to choose $L$ for an observer?](https://mediaspace.illinois.edu/media/t/1_6kd1vo4k/292883062)
    * [Do observers break controllers?](https://mediaspace.illinois.edu/media/t/1_93pesfka/292883062)
    * [When is observer design possible?](https://mediaspace.illinois.edu/media/t/1_rxgahknr/292883062)

#### Monday, March 20 (Third design project)

* Read the [guidelines for the third design project](projects#design-project-3-spacecraft-with-star-tracker).
* Complete [HW17 in PrairieLearn](https://us.prairielearn.com/pl/course_instance/130996/assessment/2327068).

#### Wednesday, March 22 (More observer design and analysis)

* Read the [notes on observers (part 2) from class](notes/20230322-observers-part-2.pdf).
* Complete [HW18 in PrairieLearn](https://us.prairielearn.com/pl/course_instance/130996/assessment/2327069).

#### Friday, March 24 (Optimal observers, part 1)

* Read the [notes on optimal observers from class](notes/20230324-optimal-observers.pdf).

#### Monday, March 27 (Optimal observers, part 2)

* Read the [notes on optimal observers from class](notes/20230324-optimal-observers.pdf).
* Complete [HW19 in PrairieLearn](https://us.prairielearn.com/pl/course_instance/130996/assessment/2327071).

#### Wednesday, March 29 (Optimal observers, part 3)

* Read the [notes on optimal observers from class](notes/20230324-optimal-observers.pdf).
* (Optional) Watch supplementary videos on optimal observers:
    * [What is an optimal observer?](https://mediaspace.illinois.edu/media/t/1_m5ku4a2i/292883062)
    * [What problem is solved to produce an optimal observer?](https://mediaspace.illinois.edu/media/t/1_cvfqo0i0/292883062)
    * [Do optimal observers make any sense at all?](https://mediaspace.illinois.edu/media/t/1_axx6h2yg/292883062)

#### Friday, March 31 (No class)

* Submit [DP3 Draft 2](projects#draft-report-with-results-by-1159pm-on-friday-march-31) by midnight today.
* Have fun at EOH! I will hold an extra office hour during our normal class time.

## Part 4 - Tracking and frequency response

#### Monday, April 3 (Random number generation)

* See [example code]({{ site.github.repository_url }}/tree/main/examples/20230403).

#### Wednesday, April 5 (Tracking, part 1)

* See [example code]({{ site.github.repository_url }}/tree/main/examples/20230405).

#### Friday, April 7 (Tracking, part 2)

* See [example code]({{ site.github.repository_url }}/tree/main/examples/20230407).

#### Monday, April 10 (Fourth design project)

* Read the [guidelines for the third design project](projects#design-project-4-drone).

#### Wednesday, April 12 (Frequency response, part 1)

* Complete [HW20 in PrairieLearn](https://us.prairielearn.com/pl/course_instance/130996/assessment/2341242).

#### Friday, April 14 (Frequency response, part 2)

* Complete [HW21 in PrairieLearn](https://us.prairielearn.com/pl/course_instance/130996/assessment/2327072).

#### Monday, April 17 (Transfer function)

* Read the [notes on transfer functions from class](notes/20230417-transfer-function.pdf)
* Complete [HW22 in PrairieLearn](https://us.prairielearn.com/pl/course_instance/130996/assessment/2327073).
* (Optional) Watch supplementary videos on frequency response:
    * [What is the transfer function? What is the frequency response? Why are these things important?](https://mediaspace.illinois.edu/media/t/1_oljuanym/292883062)
    * [What is the solution to a system with input?](https://mediaspace.illinois.edu/media/t/0_29rmp7ql/292883062)
    * [What is a complex number?](https://mediaspace.illinois.edu/media/t/1_h4sfjfwc/292883062)
    * [How do I derive the transfer function?](https://mediaspace.illinois.edu/media/t/1_xzp809m6/292883062)
    * [How do I derive the frequency response?](https://mediaspace.illinois.edu/media/t/1_xmptsbkg/292883062)
* See [example code]({{ site.github.repository_url }}/tree/main/examples/20230417).

#### Wednesday, April 19 (Bode plot)

* Read the [notes on Bode plots from class](notes/20230419-bode-plot.pdf).
* See [example code]({{ site.github.repository_url }}/tree/main/examples/20230419).

#### Friday, April 21 (Bandwidth + Computation time)

* See [example code]({{ site.github.repository_url }}/tree/main/examples/20230421).

#### Monday, April 24 (No class)

* Good luck on Exam 4!

#### Wednesday, April 26 (Collision avoidance)

* Read the [notes on collision avoidance from class](notes/20230426-collision-avoidance.pdf)
* See [example code]({{ site.github.repository_url }}/tree/main/examples/20230426).


# Examples

* See [examples folder of the ae353 github repository]({{ site.github.repository_url }}/tree/main/examples)
