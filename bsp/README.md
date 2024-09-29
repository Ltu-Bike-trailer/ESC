# Boar-support package

This crate provides a simple board-support package.
It defines all of the needed abstraction to use the board in the `control-software` repository.
This includes pin mappings event mappings and simple abstractions to restrict hardware usage.

Design goals:

- Allow pin changes without modifying the code that relies on this.
- Allow event handling in user code without knowing exactly what pin is responsible for what.
- Contain a minimal set of code as to make it easy to maintain.
- Use type-system to ensure that the pin configurations happen once and in the correct order.

To the end of the last point one needs to use const <FLAGNAME>:bool to ensure that the flag is not set when the function is called and return where the
flag is set. This makes it easy to know if the function has been called or not and ensures that the code will not allow the user to call the same function twice.
