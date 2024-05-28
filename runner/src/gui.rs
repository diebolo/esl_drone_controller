use std::process::{exit, Stdio};
/// Launches a specified Python script as a child process and returns the handle to this process.
///
/// # Parameters
///
/// * `python_script` - A string containing the path to the Python script, located at "runner/src/demo.py".
///
/// # Returns
///
/// Returns a `std::process::Child` type object, representing the launched child process. If the process fails to start,
/// the function will print an error message and exit the program.
pub fn start_interface() ->std::process::Child{
    //define the path of python file
    let python_script = "runner/src/demo.py";
    // create a child process to cooperate the python file
    let child = match std::process::Command::new("python3")
        .arg(python_script)
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .spawn()
    {
        Ok(child) => child,
        Err(e) => {
            eprintln!("can not start python process: {}", e);
            exit(1);
        }
    };
    child
}