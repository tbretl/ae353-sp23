---
layout: default
title: Reference
tagline: An invitation to aerospace control
description: Supplementary notes and other reference material
---

## Contents
{:.no_toc}

* This text will be replaced by a table of contents (excluding the above header) as an unordered list
{:toc}

---

## PID

The **proportional-integral-derivative** or **PID** controller is a simple, widely-used, and often very effective way to make a dynamic system do what you want.

Suppose this dynamic system has exactly one real-valued **input** $u(t)$ and one real-valued **output** $y(t)$. Suppose our goal is to choose the input $u(t)$ so that the output $y(t)$ converges to a desired **reference** value $r$. Define the **error** as

$$
e(t) = y(t) - r.
$$

The PID controller is

$$
u(t) = - k_p e(t) - k_i \int_0^t e(\tau) d\tau - k_d \frac{d e(t)}{dt}
$$

for some choice of constant **gains** $k_p$, $k_i$, and $k_d$. The first term in this controller is "proportional feedback," the second term is "integral feedback," and the third term is "derivative feedback."

#### How to compute the proportional feedback term

We assume the controller is given $r$ and has access to a measurement of $y(t)$, so computing the term

$$
- k_p e(t)
$$

is just arithmetic.

#### How to compute the integral feedback term

To compute the term

$$
- k_i \int_0^t e(\tau) d\tau
$$

we need to find the integral

$$
\int_0^t e(\tau) d\tau
$$

at the current time $t$. In most cases, we cannot find this integral exactly even if we wanted to, because we do not know $y(t)$ --- and, therefore, $e(t)$ --- at *all* times $t \in \mathbb{R}$, but rather only at *sampled* times

$$
t \in \{ 0, \Delta t, 2\Delta t, 3\Delta t, \dotsc \}
$$

for some time step $\Delta t > 0$. For this reason, it is common to use the right-hand rectangle method of numerical integration:

$$
\int_0^t e(\tau) d\tau \approx \int_0^{t - \Delta t} e(\tau) d\tau + \Delta t \cdot e(t).
$$

Here is another way of thinking about this. Suppose we define a new variable

$$
z(t) = \int_0^t e(\tau) d\tau
$$

to contain the integral of the error. It is equivalent to say that $z(t)$ is described by the first-order ordinary differential equation

$$
\frac{d z(t)}{dt} = e(t)
$$

with the initial condition

$$
z(0) = 0.
$$

Then, it is common to use the **backward Euler method**

$$
z(t) \approx z(t - \Delta t) + \Delta t \cdot e(t)
$$

to compute an approximation to $z(t)$. Applying the backward Euler method to find $z(t)$ is exactly the same as applying the right-hand rectangle method to find the integral that $z(t)$ represents.

To implement the backward Euler method with code, you need to define a variable to store the prior value $z(t - \Delta t)$ of the error integral, and you need to update the value of this variable at each time step. You will have to initialize the error integral to something when you start your controller --- it is common to start with a value of zero.

#### How to compute the derivative feedback term

To compute the term

$$
- k_d \frac{d e(t)}{dt}
$$

we need to find the derivative

$$
\frac{d e(t)}{dt}
$$

at the current time $t$. Just like for the integral term, we usually cannot find this derivative exactly. So, just like it is common to use the backward Euler method to find the integral of the error, it is common to use the **backward finite difference method** to find the derivative of the error:

$$
\frac{d e(t)}{dt} \approx \frac{e(t) - e(t - \Delta t)}{\Delta t}.
$$

To implement the backward finite difference method with code, you need to define a variable to store the prior value $e(t - \Delta t)$ of the error, and you need to update the value of this variable at each time step. Again, you will have to initialize this prior error to something when starting your controller --- it is common to start with a value of zero.




## State space models

#### What is a state space model?

A **state-space model** is a set of ordinary differential equations that can be written in this form:

$$
\dot{x} = Ax + Bu
$$

There are two variables:

* $x$ is the **state**
* $u$ is the **input**

Both variables are functions of time, so we could have written $x(t)$ and $u(t)$ --- but in general, we won't use that notation unless we need it. We use dots to indicate time derivatives, so $\dot{x}$ means $dx(t)/dt$.

The state and input may have more than one element --- that is to say, they may be vectors (represented by a matrix of numbers) and not scalars (represented by a single number). Suppose, in particular, that:

* the state $x$ has $n_x$ elements
* the input $u$ has $n_u$ elements

Then, we would represent them as column matrices:

$$
x =
\begin{bmatrix}
x_1 \\ \vdots \\ x_{n_x}
\end{bmatrix}
\qquad
u =
\begin{bmatrix}
u_1 \\ \vdots \\ u_{n_u}
\end{bmatrix}
$$

Taking the time derivative of a matrix is equivalent to taking the time derivative of each element of that matrix, so we would write

$$
\dot{x} =
\begin{bmatrix}
\dot{x}_1 \\ \vdots \\ \dot{x}_{n_x}
\end{bmatrix}
$$

The state-space model also has two constants: $A$ and $B$. If $x$ and $u$ are column matrices with (possibly) more than one element, then these constants have to be matrices as well:

* $A$ is a constant matrix of size $n_x \times n_x$
* $B$ is a constant matrix of size $n_x \times n_u$

The state space model has two key properties:

* it is **linear** because both $\dot{x}$ is a linear function of $x$ and $u$
* it is **time-invariant** because $A$ and $B$ are constant

<div class="alert alert-warning">
Other people use other symbols for both the variables and constants in a state space model. Indeed, we will sometimes use other symbols for these things as well. For example:

$$
\dot{z} = Ez + Fv
$$

This is also a "state space model," in which the state is $z$ and the input is $v$.
</div>


#### How do I put a system in state space form?

Suppose we are given a description of a dynamic system. We would like to describe this same system with a state space model.

Remember that every state space model is linear. The equations of motion that describe a dynamic system are often nonlinear. So apparently, we will have to accept that state space models can only *approximate* some of the systems we want to describe.

We will use **linearization** to arrive at this approximation, in four steps.

**Step 1.** Rewrite the system as a set of first-order ordinary differential equations:

$$
\dot{m} = f(m,n)
$$

In this expression, the variables $m$ and $n$ are functions of time and can have more than one element --- in general, you should represent them as column matrices. As we said before, the function $f(\cdot)$ will often be nonlinear.

**Step 2.** Find an equilibrium point $m_{e}, n_{e}$ of the system by solving this equation:

$$
0 = f(m_{e},n_{e})
$$

A solution to this equation is called an equilibrium point because if

$$m = m_e \qquad n = n_e$$

then

$$\dot{m} = f(m_e, n_e) = 0$$

and so $m$ remains constant. That is to say, if the system reaches an equilibrium point, then it stays there. This is an important property! The goal of almost every controller we design in this course will be to make a system reach an equilibrium point quickly and reliably.

This equation may have no solutions, in which case no equilibrium point exists for the system. This is bad. We will ignore this situation for now.

This equation also may have many solutions, in which case you have to make a choice. A good choice is whatever equilibrium point you would like your system to achieve.

**Step 3.** Define the state and input as follows:

$$
x = m-m_{e}
\qquad
u = n-n_{e}
$$

Note that $x$ measures *error* --- the *difference* between $m$ and its equilibrium value. When error is zero, the system has reached equilibrium and is doing what we want. Apparently, with this way of defining the state and the input, "control design" means choosing $u$ so that $x$ goes to zero.

**Step 4.** Compute $A$ and $B$ as follows:

$$
A = \frac{\partial f}{\partial m}\biggr\rvert_{(m_{e},n_{e})}
\qquad
B = \frac{\partial f}{\partial n}\biggr\rvert_{(m_{e},n_{e})}
$$

<div class="alert alert-warning">
Recall that

$$
\frac{\partial f}{\partial m}\biggr\rvert_{(m_{e},n_{e})}
$$

is the <a href="https://en.wikipedia.org/wiki/Jacobian_matrix_and_determinant">Jacobian (i.e., matrix of partial derivatives)</a> of $f$ with respect to $m$, evaluated at the equilibrium point.
</div>

Why does this make any sense? Look again at the ODEs that describe the original system:

$$\dot{m} = f(m, n)$$

First, because

$$x = m - m_e$$

then

$$\dot{x} = \dot{m} - 0 = \dot{m}$$

So the *left*-hand side of the ODEs can simply be replaced with $\dot{x}$.

Now, suppose we want a linear approximation to the *right*-hand side of the ODEs --- the function $f(m, n)$. One way to find such an approximation is to take a [Taylor's series expansion](https://en.wikipedia.org/wiki/Taylor_series) about $m_e, n_e$ up to first order:

$$
\begin{aligned}
f(m, n)
&\approx f(m_e, n_e) + \frac{\partial f}{\partial m}\biggr\rvert_{(m_{e},n_{e})} \left( m - m_e \right) + \frac{\partial f}{\partial n}\biggr\rvert_{(m_{e},n_{e})} \left( n - n_e \right) \\
&= 0 + A x + B u \\
&= A x + B u
\end{aligned}
$$

There you have it: "$\dot{x} = Ax + Bu$" is a first-order (i.e., linear) approximation to "$\dot{m} = f(m, n)$".


##### Example (first-order)

Consider the system with dynamics that are described by the following equation of motion:

$$\dot{\omega} + 2 \omega = \tau$$

Let's apply our method to put this system in state space form.

We begin by rewriting it as a set of first-order ODEs. Lucky us, the system is already described by just one first-order ODE, so all we need to do is solve for $\dot{w}$:

$$
\dot{\omega} = f(\omega, \tau) = -2\omega + \tau
$$

Next, we find an equilibrium point by solving

$$0 = -2 \omega_e + \tau_e$$

In this case, there are many solutions. Suppose we pick this one:

$$\omega_e = 10 \qquad \tau_e = 20$$

Then, we define the state and input based on this choice of equilibrium point:

$$x = \omega - \omega_e \qquad u = \tau - \tau_e$$

Finally, we compute $A$ and $B$ by taking Jacobians (easy in this case because all the variables are scalars):

$$
\begin{aligned}
A &= \frac{\partial \left(-2\omega + \tau\right)}{\partial \omega}\biggr\rvert_{(\omega_{e},\tau_{e})} = -2 \\[1em]
B &= \frac{\partial \left(-2\omega + \tau\right)}{\partial \tau}\biggr\rvert_{(\omega_{e},\tau_{e})} = 1
\end{aligned}
$$

The resulting state space model is

$$
\begin{aligned}
\dot{x} &= Ax + Bu \\
&= -2x + u
\end{aligned}
$$

Note that the original system was linear, so there was no approximation here. We could probably have skipped most of these steps and written the system in state-space form by inspection. On the other hand, it is nice to know that the process of "linearization" still works even in this simple case.


##### Example (second-order)

Consider the system with dynamics that are described by the following equation of motion:

$$\ddot{q} + 3 \sin q = \tau$$

Let's apply our method to put this system in state-space form.

We begin by rewriting it as a set of first-order ODEs:

* We find the time-derivative of $q$ with highest order --- in this case, $\ddot{q}$, of order 2.
* We define *new variables* for each time-derivative of $q$ with lower order --- in this case, $\dot{q}$, the only time-derivative with order between 0 and 2. We might choose the following name for this time-derivative:

$$ v = \dot{q} $$

* We rewrite the original ODEs (in this case, just one) in terms of these new variables:

$$\dot{v} + 3\sin q = \tau$$

* We add one extra ODE for each new variable (in this case, just one extra) --- this is trivial, coming from the way we defined these new variables:

$$\dot{q} = v$$

* We collect the original ODEs and the extra ODEs together, if necessary solving for all of the time-derivatives (that's not necessary here):

$$
\begin{aligned}
\dot{q} &= v \\
\dot{v} &= -3\sin q + \tau
\end{aligned}
$$

* Finally, we rewrite our result in the form $\dot{m} = f(m, n)$ by collecting things in column matrices as follows:

$$\begin{bmatrix} \dot{q} \\ \dot{v} \end{bmatrix} = \begin{bmatrix} v \\ -3\sin q + \tau \end{bmatrix}$$

Note that, as desired, these rewritten ODEs have time derivatives that are at most of first order. Also note that all of these time-derivatives are on the left-hand side of the equations --- none appear on the right-hand side.

Next, we find an equilibrium point by solving

$$\begin{bmatrix} 0 \\ 0 \end{bmatrix} = \begin{bmatrix} v \\ -3\sin q + \tau \end{bmatrix}$$

There are many solutions. Suppose we pick this one:

In this case, there are many solutions. Suppose we pick this one:

$$q_e = \pi / 2 \qquad v_e = 0 \qquad \tau_e = 3$$

Then, we define the state and input based on this choice of equilibrium point:

$$x = \begin{bmatrix} q - q_e \\ v - v_e \end{bmatrix} \qquad u = \begin{bmatrix}\tau - \tau_e\end{bmatrix}$$

Finally, we compute $A$ and $B$ by taking Jacobians.

$$
\begin{aligned}
A
&= \frac{\partial f}{\partial \begin{bmatrix} q \\ v \end{bmatrix}}\biggr\rvert_{\left(\begin{bmatrix}q_e\\v_e\end{bmatrix},\begin{bmatrix}\tau_{e}\end{bmatrix}\right)} \\
&= \left.\begin{bmatrix} \dfrac{\partial(v)}{\partial q} & \dfrac{\partial(v)}{\partial v} \\ \dfrac{\partial(-3\sin q + \tau)}{\partial q} & \dfrac{\partial(-3\sin q + \tau)}{\partial v} \end{bmatrix}\right\rvert_{\left(\begin{bmatrix}q_e\\v_e\end{bmatrix},\begin{bmatrix}\tau_{e}\end{bmatrix}\right)} \\
&= \left.\begin{bmatrix} 0 & 1 \\ -3\cos q & 0 \end{bmatrix}\right\rvert_{\left(\begin{bmatrix}q_e\\v_e\end{bmatrix},\begin{bmatrix}\tau_{e}\end{bmatrix}\right)} \\
&= \begin{bmatrix} 0 & 1 \\ 0 & 0 \end{bmatrix} \\[1em]
B
&= \frac{\partial f}{\partial \begin{bmatrix} \tau \end{bmatrix}}\biggr\rvert_{\left(\begin{bmatrix}q_e\\v_e\end{bmatrix},\begin{bmatrix}\tau_{e}\end{bmatrix}\right)} \\
&= \left.\begin{bmatrix} \dfrac{\partial(v)}{\partial \tau} \\ \dfrac{\partial(-3\sin q + \tau)}{\partial \tau} \end{bmatrix}\right\rvert_{\left(\begin{bmatrix}q_e\\v_e\end{bmatrix},\begin{bmatrix}\tau_{e}\end{bmatrix}\right)} \\
&= \left.\begin{bmatrix} 0 \\ 1 \end{bmatrix}\right\rvert_{\left(\begin{bmatrix}q_e\\v_e\end{bmatrix},\begin{bmatrix}\tau_{e}\end{bmatrix}\right)} \\
&= \begin{bmatrix} 0 \\ 1 \end{bmatrix}
\end{aligned}
$$

The resulting state space model is

$$
\begin{aligned}
\dot{x} &= Ax + Bu \\
&= \begin{bmatrix} 0 & 1 \\ 0 & 0 \end{bmatrix}x + \begin{bmatrix} 0 \\ 1 \end{bmatrix}u
\end{aligned}
$$

The original system was nonlinear and the state space model is linear, so there *must* be some approximation here! As we will see, this approximation is good near the equilibrium point but can be very bad elsewhere.


## The matrix exponential function

#### What is the matrix exponential?

The **matrix exponential** of a square matrix $M$ is the infinite series

$$
e^M = I + M + \frac{1}{2}M^2 + \frac{1}{6}M^3 + \dotsm = \sum_{k=0}^{\infty} \dfrac{1}{k !} M^k
$$

where $I$ is the identity matrix. This series converges for any square matrix $M$.


#### How do I use the matrix exponential to solve a linear system?

The solution to the set of linear ODEs

$$ \dot{x} = Fx $$

with the initial condition

$$ x(t_0) = x_0 $$

is

$$ x(t) = e^{F (t - t_0)} x_0. $$

How do we know that this solution is correct? First, let's check that this solution satisfies the ODEs:

$$
\begin{aligned}
\dot{x}
&= \frac{d}{dt} \left( e^{F (t - t_0)} x_0 \right) \\
&= \frac{d}{dt} \left( \left( I + F(t-t_0) + \frac{1}{2} F^2(t-t_0)^2 + \frac{1}{6} F^2(t-t_0)^3 + \dotsm \right) x_0 \right) \\
&= \frac{d}{dt} \left( I + F(t-t_0) + \frac{1}{2} F^2(t-t_0)^2 + \frac{1}{6} F^3(t-t_0)^3 + \dotsm \right) x_0 \\
&= \left( \frac{d}{dt} \left( I \right) + \frac{d}{dt} \left( F(t-t_0) \right) + \frac{d}{dt} \left( \frac{1}{2} F^2(t-t_0)^2 \right) + \frac{d}{dt} \left( \frac{1}{6} F^3(t-t_0)^3 \right) \dotsm \right) x_0 \\
&= \left( 0 + F + F^2(t-t_0) + \frac{1}{2} F^3(t-t_0) + \dotsm \right) x_0 \\
&= F \left(I + F(t-t_0) + \frac{1}{2} F^2(t-t_0)^2 + \dotsm \right) x_0 \\
&= F e^{F(t-t_0)} x_0
\end{aligned}
$$

Apparently, it does. Second, let's check that this solution satisfies the initial condition:

$$ \begin{aligned} x(t_0) &= e^{F(t_0 - t_0)} x_0 \\ &= e^0 x_0 \\ &= I x_0 \\ &= x_0 \end{aligned} $$

Again, it does. (We might wonder if this is the *only* solution to the original ODEs --- it is, although a proof would require more work.)

#### How do I use the matrix exponential to solve state space models?

Consider the state space model

$$ \dot{x} = Ax + Bu$$

This model does not look the same as

$$ \dot{x} = Fx $$

Indeed, without specifying $u$, we cannot solve for $x$ as a function of time. However, particular choices of $u$ allow us to simplify the state space model. For example, if we choose $u = 0$, then we can write

$$ \begin{aligned} \dot{x} &= Ax+Bu \\ &= Ax + B \cdot (0) \\ &= Ax + 0 \\ &= Ax \end{aligned} $$

and so we are right back at a linear system that can be solved with the matrix exponential. Another common choice of $u$ is

$$ u = -Kx $$

for some constant matrix $K$. (What would the size of $K$ have to be for us to define $u$ in this way?) This choice of $u$ is called **state feedback**, since the input depends on the state. If we plug this choice into our state space model, then we can write

$$
\begin{aligned}
\dot{x} &= Ax + Bu \\
&= Ax + B(-Kx) \\
&= (A - BK) x
\end{aligned}
$$

and so --- just like before --- we are right back at a linear system that can be solved with the matrix exponential. Although this result will get us a long way, we will see how to solve state space models for other choices of input later on.

<div class="alert alert-warning">
Given the state-space model

$$\dot{x} = Ax + Bu$$

it is standard to call the system

$$\dot{x} = Ax$$

that results from the application of zero input $u=0$ the <strong>open-loop system</strong>. Similarly, it is standard to call the system

$$\dot{x} = (A - BK) x$$

that results from the application of linear state feedback $u = -Kx$ the <strong>closed-loop system</strong>. Remember that "zero input" is not necessarily the same as "zero actuator commands." When <a href="#how-do-i-put-a-system-in-state-space-form">linearizing equations of motion to derive a state space model</a>, we defined

$$u = n - n_e$$

where $n$ was the set of actuator commands and $n_e$ was the value of these commands at equilibrium. So,

$$u = 0$$

actually means

$$n = n_e.$$

Similarly,

$$u = -Kx$$

actually means

$$n = n_e - Kx = n_e - K(m - m_e).$$

The term $n_e$ is typically referred to as <strong>feedforward</strong> and the term $-K(m - m_e)$ is typically referred to as <strong>feedback</strong>.
</div>


## Asymptotic stability

#### What are eigenvalues and eigenvectors?

Consider a square matrix $F \in \mathbb{R}^{n \times n}$. If we can find a complex number $s \in \mathbb{C}$ and a non-zero, complex-valued vector $v \in \mathbb{C}^n$ that satisfy

$$ (s I - F) v = 0 $$

then we call $s$ an **eigenvalue** of $F$ and $v$ the corresponding **eigenvector** of $F$. If we wanted, we could rearrange this equation to put it in a form that might be more familiar:

$$ 0 = (s I - F) v = s v - F v \qquad\Rightarrow\qquad F v = s v. $$

One way to find eigenvalues is to solve the equation

$$ \det (s I - F) = 0 $$

where "$\det(\cdot)$" means taking a determinant. In general, this equation will be an $n$th-order polynomial in $s$, and so will have $n$ solutions --- we might call them $s_1, \dotsc, s_n$. To find an eigenvector that corresponds to each eigenvalue $s_i$, we solve

$$ F v_i = s_i v_i $$

for $v_i$. Note that there are many possible solutions to this equation and that eigenvectors are only unique up to a scale factor. In particular, for any real number $k \in \mathbb{R}$, we have

$$ \begin{aligned} F (k v_i) &= k (F v_i) \\ &= k (s v_i) \\ &= s (k v_i). \end{aligned}$$

Apparently, if $v_i$ is an eigenvector corresponding to $s_i$, then so is $k v_i$ for any $k \neq 0$. For this reason, algorithms to find eigenvectors typically *normalize* them to have unit length.

#### How do I diagonalize a square matrix?

Suppose we have found the eigenvalues $s_1, \dotsc, s_n$ and eigenvectors $v_1, \dotsc, v_n$ of a square matrix $F\in\mathbb{R}^{n \times n}$. Define the matrix

$$V = \begin{bmatrix} v_1 & \dotsm & v_n \end{bmatrix}$$

with an eigenvector in each column, and also the matrix

$$\text{diag} (s_1, \dotsc, s_n) = \begin{bmatrix} s_1 & 0 & \dotsm & 0 \\ 0 & s_2 & \dotsm & 0 \\ \vdots & \vdots & \ddots & \vdots \\ 0 & 0 & \dotsm & s_n \end{bmatrix}$$

with the eigenvalues along the diagonal.

Two things are true.

First, the following equality holds:

$$F V = V \text{diag} (s_1, \dotsc, s_n) $$

You could easily verify this result for yourself.

Second, if $s_1, \dotsc, s_n$ are all *distinct* (i.e., if no two eigenvalues are the same), then the matrix $V$ is *invertible*. This result is harder to verify --- it has to do with the fact that if the eigenvalues are distinct then the eigenvectors are linearly independent.

The key consequence of $V$ being invertible is that we can solve the above equality to write:

$$\text{diag} (s_1, \dotsc, s_n) = V^{-1} F V.$$

In this case --- if all eigenvalues are distinct and so the matrix of eigenvectors is invertible --- we say that $F$ is **diagonalizable**. The process of "diagonalizing $F$" is finding the matrix $V$.

#### What is the matrix exponential of a diagonal matrix?

It is easy to find the matrix exponential of a diagonal matrix, starting from [the definition](#what-is-the-matrix-exponential):

$$
\begin{align*}
e^{\text{diag} (s_1, \dotsc, s_n)t}
&= I + \text{diag} (s_1, \dotsc, s_n)t + \frac{1}{2}\left( \text{diag} (s_1, \dotsc, s_n) t \right)^2 + \dotsm \\
&=
\begin{bmatrix} 1 & 0 & \dotsm & 0 \\ 0 & 1 & \dotsm & 0 \\ \vdots & \vdots & \ddots & \vdots \\ 0 & 0 & \dotsm & 1 \end{bmatrix}
+
\begin{bmatrix} s_1t & 0 & \dotsm & 0 \\ 0 & s_2t & \dotsm & 0 \\ \vdots & \vdots & \ddots & \vdots \\ 0 & 0 & \dotsm & s_nt \end{bmatrix}
+
\begin{bmatrix} (s_1t)^2/2 & 0 & \dotsm & 0 \\ 0 & (s_2t)^2/2 & \dotsm & 0 \\ \vdots & \vdots & \ddots & \vdots \\ 0 & 0 & \dotsm & (s_nt)^2/2 \end{bmatrix}
+ \dotsm \\
&= \begin{bmatrix} 1 + s_1t + (s_1t)^2/2 + \dotsm & 0 & \dotsm & 0 \\ 0 & 1 + s_2t + (s_2t)^2/2 + \dotsm & \dotsm & 0 \\ \vdots & \vdots & \ddots & \vdots \\ 0 & 0 & \dotsm & 1 + s_nt + (s_nt)^2/2 + \dotsm \end{bmatrix} \\
&= \begin{bmatrix} e^{s_1t} & 0 & \dotsm & 0 \\ 0 & e^{s_2t} & \dotsm & 0 \\ \vdots & \vdots & \ddots & \vdots \\ 0 & 0 & \dotsm & e^{s_nt} \end{bmatrix}
\end{align*}
$$

#### What is the solution to a linear system that is diagonalizable?

[We have seen](#how-do-i-use-the-matrix-exponential-to-solve-a-linear-system) that the solution to

$$\dot{x} = Fx$$

with the initial condition

$$x(0) = x_0$$

is

$$x(t) = e^{Ft}x_0.$$

Suppose $F$ is a diagonalizable matrix, so that

$$\text{diag} (s_1, \dotsc, s_n) = V^{-1} F V$$

where

$$\text{diag} (s_1, \dotsc, s_n)$$

is a diagonal matrix that contains the eigenvalues of $F$ and where

$$V = \begin{bmatrix} v_1 & \dotsm & v_n \end{bmatrix}$$

is a matrix of the corresponding eigenvectors. Then, applying [the definition of matrix exponential](#what-is-the-matrix-exponential) again, we have

$$
\begin{align*}
e^{Ft}x_0
&= e^{V \text{diag} (s_1, \dotsc, s_n) V^{-1}t}x_0 \\
&= \left( I + V \text{diag} (s_1, \dotsc, s_n) V^{-1}t + \frac{1}{2}\left( V \text{diag} (s_1, \dotsc, s_n) V^{-1}t \right)^2 + \dotsm\right) x_0 \\
&= V \left( I + \text{diag} (s_1, \dotsc, s_n) t + \frac{1}{2} \left( \text{diag} (s_1, \dotsc, s_n)t \right)^2 + \dotsm\right) V^{-1}x_0 \\
&= V e^{\text{diag} (s_1, \dotsc, s_n)t}V^{-1}x_0 \\
&= V \begin{bmatrix} e^{s_1t} & 0 & \dotsm & 0 \\ 0 & e^{s_2t} & \dotsm & 0 \\ \vdots & \vdots & \ddots & \vdots \\ 0 & 0 & \dotsm & e^{s_nt} \end{bmatrix} V^{-1}x_0
\end{align*}
$$

where the last step comes from what we just found out about the [matrix exponential of a diagonal matrix](#what-is-the-matrix-exponential-of-a-diagonal-matrix). In this expression, the terms $V$, $V^{-1}$, and $x_0$ are constant. The only terms that depend on $t$, in fact, are the *scalar* exponential functions

$$e^{s_1t}, \dotsc, e^{s_nt}$$

that appear in the diagonal of

$$e^{\text{diag} (s_1, \dotsc, s_n)t}.$$

Therefore, we can infer the behavior of $x(t)$ based entirely on these scalar exponential functions. In particular, suppose that each eigenvalue $s_i$ — a complex number — has real part $a_i$ and imaginary part $b_i$, or in other words that

$$s_i = a_i + jb_i.$$

[Euler's formula](https://en.wikipedia.org/wiki/Euler%27s_formula) tells us that

$$e^{s_it} = e^{(a_i+jb_i)t} = e^{a_it}\left(\cos(b_it) + j\sin(b_it)\right).$$

Apparently, as time gets large, one of three things is true about each of these terms:

* if $a_i > 0$, then $e^{(a_i+jb_i)t}$ grows quickly
* if $a_i = 0$, then $e^{(a_i+jb_i)t}$ is constant ($b_i=0$) or is sinusoidal with constant magnitude ($b_i \neq 0$)
* if $a_i < 0$, then $e^{(a_i+jb_i)t}$ decays quickly to zero

It is possible to show that (more or less) the same result holds for *any* system $\dot{x}=Fx$, not only ones for which $F$ is diagonalizable. This takes more work, and involves the transformation of $F$ into [Jordan normal form](https://en.wikipedia.org/wiki/Jordan_normal_form) rather than into a diagonal matrix. We would discover that the terms that depend on $t$ all look like

$$t^me^{s_it}$$

where $m$ is an integer that is at most the multiplicity of the eigenvalue $s_i$. Since $e^{a_it}$ increases or decreases a lot faster than $t^m$, then the same three things we listed above would be true of each term in $x(t)$, just like before.

See [the reference textbook](https://fbswiki.org/) for details.

#### When is a linear system asymptotically stable?

The system

$$\dot{x} = Fx$$

is called **asymptotically stable** if $x(t) \rightarrow 0$ as $t \rightarrow \infty$, starting from any initial condition $x(t_0) = x_0.$

Based on our observations about [the solution to linear systems that are diagonalizable](#what-is-the-solution-to-a-linear-system-that-is-diagonalizable), we can state the following important result:

<div class="alert alert-warning">
The system

$$\dot{x} = Fx$$

is asymptotically stable if and only if all eigenvalues of $F$ have negative real part.
</div>

In particular, we now have a test for whether or not a controller "works." Suppose we apply linear state feedback

$$u = -Kx$$

to the state space system

$$\dot{x} = Ax + Bu$$

so that

$$\dot{x} = (A - BK)x.$$

The controller "works" when this system is asymptotically stable, i.e., when $x$ goes to zero as time gets large. We now know, therefore, that the controller "works" when all eigenvalues of $A - BK$ have negative real part.

We may not have a systematic way of *finding* a matrix $K$ to make the closed-loop system stable yet, but we certainly do have a systematic way now of deciding whether or not a *given* matrix $K$ makes the closed-loop system stable.




## Eigenvalue placement by controllable canonical form

Apply the input

$$u=-Kx$$

to the open-loop system

$$\dot{x} = Ax+Bu $$

and you get the closed-loop system

$$\dot{x} = (A-BK)x.$$

Suppose we want to choose $K$ to put the eigenvalues of the closed-loop system, i.e., the eigenvalues of the matrix $A-BK$, at given locations. We will derive a formula called **Ackermann's method** that allows us to do this when possible, and will show how to decide when doing so is impossible.

<div class="alert alert-danger">
<p>
This entire discussion will be based on the assumption that there is exactly one input (i.e., that $u$ is a vector of length $1$, or that $n_u = 1$). Ackermann's method, in particular, <em>cannot be used</em> for eigenvalue placement when there is more than one input.
</p>

However, other similar methods — for example, the method of Tits and Yang (<a href="https://doi.org/10.1109/9.539425">"Globally convergent algorithms for robust pole assignment by state feedback," <em>IEEE Transactions on Automatic Control</em>, 41:1432-1452, 1996)</a>, as implemented by <a href="https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.place_poles.html#scipy-signal-place-poles">scipy.signal.place_poles</a> in python — <em>can</em> be used when there are multiple inputs. Our result about when eigenvalue placement is possible — i.e., about when a system is "controllable" — also generalizes to systems with multiple inputs, although it becomes harder to prove.
</div>

#### Eigenvalues are invariant to coordinate transformation


Consider the system

$$\dot{x} = Ax+Bu.$$

Suppose we define a new state variable $z$ so that

$$x = Vz$$

for some invertible matrix $V$, and so

$$\dot{x} = V\dot{z}$$

by differentiation. We have called this process "coordinate transformation" — it is exactly the same process we used for diagonalization when establishing our result about asymptotic stability.
Plug these two things into our original state-space model and we get


$$V\dot{z} = AVz+Bu.$$

Solve for $\dot{z}$ and we get the equivalent state-space model

$$\dot{z} = V^{-1}AV z + V^{-1}B u.$$

Finding a solution $z(t)$ to this transformed system allows us to recover a solution

$$x(t) = Vz(t)$$

to the original system. We would like to know if these two solutions "behave" the same way. In particular, we would like to know if the eigenvalues of $A$ are the same as the eigenvalues of $V^{-1}AV$.

First, let's look at the eigenvalues of $A$. We know that they are the roots of

$$\det\left(sI-A\right).$$

Second, let's look at the eigenvalues of $V^{-1}AV$. We know that they are the roots of

$$\det\left( sI-V^{-1}AV \right)$$

We can play a trick.
Notice that

$$V^{-1}(sI)V = s V^{-1}V = sI$$

and so

$$
\begin{align*}
\det\left( sI-V^{-1}AV \right)
&= \det\left( V^{-1}(sI)V-V^{-1}AV \right) && \text{because of our trick} \\
&= \det\left( V^{-1}(sI - A) V \right).
\end{align*}
$$

It is a fact that

$$\det(MN)=\det(M)\det(N)$$

for any square matrices $M$ and $N$.
Applying this fact, we find

$$\det\left( V^{-1}(sI - A) V \right) = \det(V^{-1})\det(sI-A)\det(V).$$

It is another fact that

$$\det(M^{-1}) = (\det(M))^{-1} = \dfrac{1}{\det(M)}$$

Applying this other fact, we find

$$
\det(V^{-1})\det(sI-A)\det(V)
= \dfrac{\det(sI-A)\det(V)}{\det(V)}
= \det(sI-A)
$$

In summary, we have established that

$$\det(sI-A) = \det\left( sI-V^{-1}AV \right)$$

and so the eigenvalues of $A$ and $V^{-1}AV$ are the same. The consequence is, if you design state feedback for the transformed system, you'll recover the behavior you want for the original system. In particular, suppose you apply the input

$$u = -Lz$$

to the transformed system and choose $L$ to place the eigenvalues of $V^{-1}AV$ in given locations.
Applying the input

$$u = -LV^{-1}x$$

to the original system, i.e., choosing

$$K = LV^{-1},$$

will result in placing the eigenvalues of $A$ at these same locations. The reason this is important is that it is often easier to choose $L$ than to choose $K$. The process of [diagonalization](#what-is-the-solution-to-a-linear-system-that-is-diagonalizable) was important for a similar reason.

#### Controllable canonical form

In the previous section, we showed that eigenvalues are invariant to coordinate transformation. The next question is what coordinates are useful for control design. The answer to that question turns out to be something called **controllable canonical form.**

A system with $n_x$ states and $1$ input is in controllable canonical form if it looks like

$$\dot{z} = A_\text{ccf}z + B_\text{ccf} u$$

where

$$
A_\text{ccf} =
\begin{bmatrix}
-a_{1} & -a_{2} & \dotsm & -a_{n_x-1} & -a_{n_x} \\
1 & 0 & \dotsm & 0 & 0 \\
0 & 1 & \dotsm & 0 & 0 \\
\vdots & \vdots & \ddots & \vdots & \vdots \\
0 & 0 & \dotsm & 1 & 0
\end{bmatrix}
\qquad\qquad
B_\text{ccf} = \begin{bmatrix} 1 \\ 0 \\ 0 \\ \vdots \\ 0 \end{bmatrix}.
$$

Notice that

* $A_\text{ccf}$ is a matrix of size $n_x \times n_x$,
* $B_\text{ccf}$ is a matrix of size $n_x \times 1$,
* $z$ is a vector of length $n_x$, and
* $u$ is a vector of length $1$.

It is a fact that the characteristic equation of this system is given by

$$\det(sI-A_\text{ccf}) = s^{n_x} + a_{1}s^{n_x-1} + \dotsm + a_{n_x-1}s + a_{n_x}.$$

It is easy to see that this formula is true for $n_x=2$ and $n_x=3$. In particular:


* If $n_x=2$, then:

$$
\begin{align*}
\det(sI-A_\text{ccf})
&= \det\begin{bmatrix} s+a_{1} & a_{2} \\ -1 & s \end{bmatrix} \\
&= (s+a_{1})s+a_{2} = s^{2}+a_{1}s+a_{2}.
\end{align*}
$$

* If $n_x=3$, then:

$$
\begin{align*}
\det(sI-A_\text{ccf})
&= \det\begin{bmatrix} s+a_{1} & a_{2} & a_{3}\\ -1 & s & 0 \\ 0 & -1 & s \end{bmatrix} \\
&= (s+a_{1})s^{2}+a_{3}-(-a_{2}s)= s^{3}+a_{1}s^{2}+a_{2}s+a_{3}.
\end{align*}
$$

There are a variety of ways to prove that this same formula is true in general. Applying the general formula to compute the matrix determinant, for example, we would find:

$$
\begin{align*}
\det(sI-A_\text{ccf})
&= \det
\begin{bmatrix}
s+a_{1} & a_{2} & \dotsm & a_{n_x-1} & a_{n_x} \\
-1 & s & \dotsm & 0 & 0 \\
0 & -1 & \dotsm & 0 & 0 \\
\vdots & \vdots & \ddots & \vdots & \vdots \\
0 & 0 & \dotsm & -1 & s
\end{bmatrix} \\
&= (s+a_{1}) \det(T_{1})-a_{2}\det(T_{2})+a_{3}\det(T_{3})-\dotsm
\end{align*}
$$

where each matrix $T_{i}$ is upper-triangular with $-1$ in $i-1$ diagonal entries and $s$ in $n_x-i$ diagonal entries. Since the determinant of an upper-triangular matrix is the product of its diagonal entries, we have

$$
\det(T_{i}) = \begin{cases} s^{n_x-i} & \text{when $i$ is odd} \\ -s^{n_x-i} & \text{when $i$ is even} \end{cases}
$$

Plug this in, and our result follows.
Now, the reason that controllable canonical form is useful is that if we choose the input

$$
u = -Lz
$$

for some choice of gains

$$
L = \begin{bmatrix} \ell_{1} & \dotsm & \ell_{n_x} \end{bmatrix}
$$

then the "$A$ matrix" of the closed-loop system is

$$
A_\text{ccf}-B_\text{ccf}L =
\begin{bmatrix}
-a_{1}-\ell_{1} & -a_{2}-\ell_{2} & \dotsm & -a_{n_x-1}-\ell_{n_x-1} & -a_{n_x}-\ell_{n_x} \\
1 & 0 & \dotsm & 0 & 0 \\
0 & 1 & \dotsm & 0 & 0 \\
\vdots & \vdots & \ddots & \vdots & \vdots \\
0 & 0 & \dotsm & 1 & 0
\end{bmatrix}
$$

The characteristic equation of this closed-loop system, computed in the same way as for $A_\text{ccf}$, is

$$
s^{n_x} + (a_{1}+\ell_{1}) s^{n_x-1} + \dotsm + (a_{n_x-1}+\ell_{n_x-1}) s + (a_{n_x}+\ell_{n_x}).
$$

If you want this characteristic equation to look like

$$
s^{n_x} + r_{1} s^{n_x-1} + \dotsm + r_{n_x-1} s + r_{n_x}
$$

then it's obvious what gains you should choose

$$
\ell_{1} = r_{1}-a_{1} \qquad\qquad \ell_{2} = r_{2}-a_{2} \qquad\qquad \dotsm \qquad\qquad \ell_{n_x}=r_{n_x}-a_{n_x}.
$$

So, if you have a system in controllable canonical form, then it is easy to choose gains that make the characteristic equation of the closed-loop system look like anything you want (i.e., to put the closed-loop eigenvalues anywhere you want). In other words, it is easy to do control design.


#### Putting a system in controllable canonical form

We have seen that controllable canonical form is useful. Now we'll see how to put a system in this form.
Suppose we have a system

$$
\dot{x} = Ax+Bu
$$

and we want to choose an invertible matrix $V$ so that if we define a new state variable $z$ by

$$
x = Vz
$$

then we can rewrite the system as

$$
\dot{z} = A_\text{ccf}z+B_\text{ccf}u
$$

where

$$
A_\text{ccf} = V^{-1}AV
\qquad\qquad\text{and}\qquad\qquad
B_\text{ccf} = V^{-1}B
$$

are in controllable canonical form.
The trick is to look at the so-called **controllability matrix** that is associated with the transformed system:

$$
W_\text{ccf} = \begin{bmatrix} B_\text{ccf} & A_\text{ccf}B_\text{ccf} & \dotsm & A_\text{ccf}^{n_x-1}B_\text{ccf} \end{bmatrix}.
$$

We will talk more later about the controllability matrix — for now, notice that

$$
\begin{align*}
B_\text{ccf} &= V^{-1}B \\
A_\text{ccf}B_\text{ccf} &= V^{-1}AVV^{-1}B = V^{-1}AB \\
A_\text{ccf}^{2}B_\text{ccf} &= A_\text{ccf} (A_\text{ccf}B_\text{ccf}) = V^{-1}AVV^{-1}AB = V^{-1}A^{2}B \\
\vdots\qquad &= \qquad\vdots
\end{align*}
$$

You see the pattern here, I'm sure. The result is:

$$
\begin{align*}
W_\text{ccf} &= \begin{bmatrix} V^{-1}B & V^{-1}AB & \dotsm & V^{-1}A^{n_x-1}B \end{bmatrix} \\
&= V^{-1}\begin{bmatrix} B & AB & \dotsm & A^{n_x-1}B \end{bmatrix} \\
&= V^{-1}W
\end{align*}
$$

where

$$
W = \begin{bmatrix} B & AB & \dotsm & A^{n_x-1}B \end{bmatrix}
$$

is the controllability matrix associated with the original system.

There are three things to note:

* $A$ and $B$ are things that you know — you have a description of the original system, as always — so you can compute $W$.

* $A_\text{ccf}$ and $B_\text{ccf}$ are also things that you know — the values $a_1, \dotsc, a_{n_x}$ in the top row of $A_\text{ccf}$ are the coefficients of the characteristic polynomial of the matrix $A$ — so you can compute $W_\text{ccf}$.

* $W$ is a square $n_x \times n_x$ matrix — it has $n_x$ columns $B$, $AB$, and so forth, all of which have size $n_x \times 1$. So, if $W$ has non-zero determinant, then you can find its inverse.

As a consequence, you can solve for the matrix $V^{-1}$:

$$
V^{-1} = W_\text{ccf}W^{-1}.
$$

Now, suppose you design a control policy for the transformed system:

$$
u = -K_\text{ccf}z.
$$

Remember, you can do this easily, because the transformed system is in controllable canonical form.
We can compute the equivalent control policy, that would be applied to the original system:

$$
u = -K_\text{ccf}z = -K_\text{ccf}V^{-1}x = -K_\text{ccf}W_\text{ccf}W^{-1}x.
$$

In particular, if we choose

$$
K = K_\text{ccf}W_\text{ccf}W^{-1}
$$

then we get the behavior that we want.
Again, we emphasize that this only works if $W$ is invertible, and that $W$ is only invertible if $\det(W)\neq 0$.

#### A systematic process for control design

Apply the input

$$
u=-Kx
$$

to the open-loop system

$$
\dot{x} = Ax+Bu
$$

and you get the closed-loop system

$$
\dot{x} = (A-BK)x.
$$

Suppose we want to choose $K$ to put the eigenvalues of the closed-loop system at

$$p_{1},\dotsc,p_{n_x}.$$

Using the results of the previous sections, we know we can do this as follows:

* Compute the characteristic equation that we want:

$$(s-p_{1})\dotsm(s-p_{n_x}) = s^{n_x}+r_{1}s^{n_x-1}+ \dotsm + r_{n_x-1}s + r_{n_x}.$$

* Compute the characteristic equation that we have:

$$\det(sI-A) = s^{n_x}+a_{1}s^{n_x-1}+ \dotsm + a_{n_x-1}s + a_{n_x}$$


* Compute the controllability matrix of the original system (and check that $\det(W)\neq 0$):

$$W = \begin{bmatrix} B & AB & \dotsm & A^{n_x-1}B \end{bmatrix}$$


* Compute the controllability matrix of the transformed system:

$$
W_\text{ccf} = \begin{bmatrix} B_\text{ccf} & A_\text{ccf}B_\text{ccf} & \dotsm & A_\text{ccf}^{n_x-1}B_\text{ccf} \end{bmatrix}
$$

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;where

$$
A_\text{ccf} =
\begin{bmatrix}
-a_{1} & -a_{2} & \dotsm & -a_{n-1} & -a_{n} \\
1 & 0 & \dotsm & 0 & 0 \\
0 & 1 & \dotsm & 0 & 0 \\
\vdots & \vdots & \ddots & \vdots & \vdots \\
0 & 0 & \dotsm & 1 & 0
\end{bmatrix}
\qquad\qquad
B_\text{ccf} = \begin{bmatrix} 1 \\ 0 \\ 0 \\ \vdots \\ 0 \end{bmatrix}
$$

* Compute the gains for the transformed system:

$$
K_\text{ccf} = \begin{bmatrix} r_{1}-a_{1} & \dotsm & r_{n_x}-a_{n_x} \end{bmatrix}
$$

* Compute the gains for the original system:

$$
K = K_\text{ccf}W_\text{ccf}W^{-1}
$$

And we're done!
This process is easy to implement, without any symbolic computation.
Remember, although this method only works for systems with exactly one input (i.e., when $n_u = 1$), similar methods work for systems with multiple inputs — in python, use <a href="https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.place_poles.html#scipy-signal-place-poles">scipy.signal.place_poles</a>.

#### How to decide when eigenvalue placement is possible

We say that a system is **controllable** if eigenvalue placement is possible. We have seen eigenvalue placement with Ackermann's method (for the special case when $n_u = 1$) is only possible when the controllability matrix $W$ is invertible. Here is a generalization of that same result to any system:


<div class="alert alert-warning">
The system

$$\dot{x} = Ax+Bu$$

is <strong>controllable</strong> if and only if
the <strong>controllability matrix</strong>

$$W = \begin{bmatrix} B & AB & \dotsm & A^{n_x-1}B \end{bmatrix}$$

is full rank, where $n_x$ is the number of states.
</div>

Let's break this statement down.

First, suppose there is only one input, so $n_u = 1$.  In that case, $A$ has size $n_x \times n_x$ and $B$ has size $n_x \times 1$. Therefore, $W$ is a square matrix of size $n_x \times n_x$, so $W$ being full rank simply means that it is invertible (i.e., that $\det(W) \neq 0$).

Now, suppose there is more than one input, so $n_u > 1$. Then, $B$ has size $n_x \times n_u$.  The matrix $W$ then has size $n_x \times n_xn_u$, and $W$ is no longer square. Although we can't invert a non-square matrix, we can still find its [rank](https://en.wikipedia.org/wiki/Rank_(linear_algebra)). In particular, it is easy to find the rank of a non-square matrix in python using [numpy.linalg.matrix_rank](https://numpy.org/doc/stable/reference/generated/numpy.linalg.matrix_rank.html#numpy-linalg-matrix-rank). We say that $W$ is **full rank** if and only if its rank is $n_x$.

If $W$ is full rank, the system is controllable, and eigenvalue placement will work. If $W$ is not full rank, the system is not controllable, and eigenvalue placement will not work.

We can actually say a little more than this. It turns out that if the controllability matrix $W$ is not full rank, then although we cannot place *all* of the system's eigenvalues, we can place *some* of these eigenvalues, while the others will remain in the same location no matter what gain matrix $K$ we choose. The eigenvalues we *can* place are often called "controllable eigenvalues" or "controllable modes," while the eigenvalues we can *not* place are often called "uncontrollable eigenvalues" or "uncontrollable modes." If the rank of $W$ is $n_x - 1$, then we can place $n_x - 1$ eigenvalues, and $1$ eigenvalue is uncontrollable. If the rank of $W$ is $n_x - 2$, then we can place $n_x - 2$ eigenvalues, and $2$ eigenvalues are uncontrollable. And so forth.