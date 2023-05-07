# Visitor Pattern

The Visitor pattern is a design pattern in object-oriented programming that allows you to separate an algorithm from an object structure on which it operates.

In the Visitor pattern, you define a new operation to perform on the elements of an object structure without changing the classes of the objects themselves. You accomplish this by creating a separate visitor class that implements the new operation for each class in the object structure. The visitor class can access the internal state of the objects it visits, allowing it to perform complex operations on them.

The main purpose of the Visitor pattern is to allow you to add new operations to an object structure without modifying the classes of the objects in that structure. This is particularly useful when you have a large and complex object hierarchy that needs to support a variety of operations, but you don't want to clutter the object classes themselves with code for every possible operation.

The Visitor pattern is commonly used in situations where you have an object structure that is fixed or stable, but you need to add new operations to it frequently. It is also useful when you have an object structure that is shared by multiple clients or applications, each of which may need to perform different operations on the objects in the structure.

Advantages of the Visitor pattern include:

- **Separation of concerns:** The Visitor pattern separates the algorithm for performing an operation on an object structure from the structure itself, which allows for greater modularity and maintainability of code.

- **Open/Closed principle:** The Visitor pattern follows the open/closed principle, which means that it allows for new operations to be added to an object structure without modifying its existing code.

- **Extensibility:** The Visitor pattern is highly extensible, as it allows new visitor classes to be added to the system without affecting existing visitor classes.

- **Encapsulation:** The Visitor pattern encapsulates operations that can be performed on objects, and allows these operations to be added and modified easily without affecting the objects themselves.

Disadvantages of the Visitor pattern include:

- **Complexity:** The Visitor pattern can add complexity to a system, especially when dealing with complex object structures and large numbers of visitors.

- **Increased coupling:** The Visitor pattern can increase coupling between objects, as each object must be aware of the visitor interface and must be able to accept a visitor object.

- **Inflexibility:** The Visitor pattern can be inflexible when it comes to modifying object structures, as any changes to the structure will require corresponding changes to the visitor classes.

Overall, the Visitor pattern can be a useful tool for separating concerns and improving the maintainability and extensibility of code, but it should be used judiciously and with an awareness of its potential drawbacks.

## Structure

![structure](img/structure-en.png)

Detailed explanation of the diagram can be found [here](https://refactoring.guru/design-patterns/visitor).

## Implementation

- [Python](../src/visitor/)