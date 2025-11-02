#![no_main]
#![no_std]

extern crate alloc;

use core::{fmt::Display, time::Duration};

use bytes::{Buf, BytesMut};
use embedded_io_async::{ErrorType, Read, Write};
use noline::builder::EditorBuilder;
use vexide::{
    io::{self, Read as _, Write as _},
    prelude::*,
    sync::RwLock,
};

pub struct VexideIO;

#[derive(Debug)]
pub struct VexideIOError(io::Error);

impl Display for VexideIOError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        self.0.fmt(f)
    }
}

impl core::error::Error for VexideIOError {}

impl embedded_io_async::Error for VexideIOError {
    fn kind(&self) -> embedded_io_async::ErrorKind {
        match self.0.kind() {
            io::ErrorKind::NotFound => embedded_io_async::ErrorKind::NotFound,
            io::ErrorKind::PermissionDenied => embedded_io_async::ErrorKind::PermissionDenied,
            io::ErrorKind::ConnectionRefused => embedded_io_async::ErrorKind::ConnectionRefused,
            io::ErrorKind::ConnectionReset => embedded_io_async::ErrorKind::ConnectionReset,
            io::ErrorKind::ConnectionAborted => embedded_io_async::ErrorKind::ConnectionAborted,
            io::ErrorKind::NotConnected => embedded_io_async::ErrorKind::NotConnected,
            io::ErrorKind::AddrInUse => embedded_io_async::ErrorKind::AddrInUse,
            io::ErrorKind::AddrNotAvailable => embedded_io_async::ErrorKind::AddrNotAvailable,
            io::ErrorKind::BrokenPipe => embedded_io_async::ErrorKind::BrokenPipe,
            io::ErrorKind::AlreadyExists => embedded_io_async::ErrorKind::AlreadyExists,
            io::ErrorKind::WouldBlock => embedded_io_async::ErrorKind::Other,
            io::ErrorKind::InvalidInput => embedded_io_async::ErrorKind::InvalidInput,
            io::ErrorKind::InvalidData => embedded_io_async::ErrorKind::InvalidData,
            io::ErrorKind::TimedOut => embedded_io_async::ErrorKind::TimedOut,
            io::ErrorKind::WriteZero => embedded_io_async::ErrorKind::WriteZero,
            io::ErrorKind::Interrupted => embedded_io_async::ErrorKind::Interrupted,
            io::ErrorKind::Other => embedded_io_async::ErrorKind::Other,
            io::ErrorKind::UnexpectedEof => embedded_io_async::ErrorKind::Other,
            _ => todo!(),
        }
    }
}

impl ErrorType for VexideIO {
    type Error = VexideIOError;
}

impl Read for VexideIO {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        let mut stdin = io::stdin().lock().await;
        let mut read = 0;

        while read == 0 {
            read += stdin.read(&mut buf[read..]).map_err(VexideIOError)?;
            sleep(Duration::from_millis(1)).await;
        }

        Ok(read)
    }
}

impl Write for VexideIO {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        let mut lock = io::stdout().lock().await;
        let mut written = 0;

        while written == 0 {
            written += lock.write(buf).map_err(VexideIOError)?;
            sleep(Duration::from_millis(1)).await;
        }

        Ok(written)
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        io::stdout().lock().await.flush().map_err(VexideIOError)
    }
}

#[vexide::main]
async fn main(peripherals: Peripherals) {
    let prompt = "> ";

    let mut io = &mut VexideIO;
    let mut editor = EditorBuilder::new_unbounded()
        .with_unbounded_history()
        .build_async(io)
        .await
        .unwrap();

    loop {
        while let Ok(line) = dbg!(editor.readline(prompt, &mut io).await) {
            println!("Read: '{}'", line);
        }

        sleep(Duration::from_millis(500)).await
    }
}
