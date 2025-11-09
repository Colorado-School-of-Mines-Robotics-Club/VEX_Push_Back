use std::{
    collections::HashMap,
    io::{self, BufRead, Read as _, Write as _, stdin},
    time::Duration,
};

use bytes::BytesMut;
use vexide::{prelude::Peripherals, time::sleep};

async fn read_line(buf: &mut String) -> io::Result<()> {
    buf.clear();

    while buf.is_empty() {
        stdin().lock().read_line(buf)?;
        sleep(Duration::from_millis(1)).await;
    }

    _ = buf.pop(); // Remove the last newline

    Ok(())
}

#[vexide::main]
async fn main(_peripherals: Peripherals) {
    let mut line = String::new();
    // let prompt = "> ";

    loop {
        // ???
        // print!("\r{}", prompt);
        // std::io::stdout().flush().unwrap();

        read_line(&mut line).await.unwrap();

        match line.as_str() {
            "help" => println!("todo"),
            _ => println!("Unknown command '{line}'"),
        }

        sleep(Duration::from_millis(10)).await
    }
}
