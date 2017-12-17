package com.example;

// the public and class words are part of the syntax.
// MyClass is the name of a "class"
public class MyClass
{
    int aSpecialNumber = 3;

    // name of myblock is "main". the name of the input to the myblock is "powers"
    // the parantheses and the curly brackets are part of the syntax
    // the curly brackets and the parantheses provide structure to the program - they
    // have no inherent meaning
    public static void main(String powers[])
    {
        System.out.println("Hello World");
        int speed = 0; // variables
        speed = speed + 1; // assignment. First the right side is calculated and put inside the
                            // left side
        String mysentence = "veer is typing real fast something";
        boolean areWehavingFun = true;
        double myFractionalNumber = 4.3;
    }

    // created a new myblock
    public void AddTwoNumbers(int number1, int number2)
    {
        int sumofTwoNumbers = 0;
        sumofTwoNumbers = number1 + number2;
    }
}
