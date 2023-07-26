use syiot::remote::{Control, ControlHandler};

struct Handler { }

impl ControlHandler<magicbox::State> for Handler {
    fn on_accept(&mut self) {
        println!(" => Accepted");
    }

    #[allow(unused_must_use)]
    fn on_msg(&mut self, msg : Result<magicbox::State, syiot::Error>) {
        dbg!(msg);
    }
}

fn main() -> Result<(), syact::Error> {
    let server = Control::new(Handler { });
    server.listen(syiot::remote::Transport::FramedTcp, "0.0.0.0:12000")?;

    server.run();

    Ok(())
}