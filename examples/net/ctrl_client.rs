use syiot::remote::ControlClient;

fn main() -> Result<(), syact::Error> {
    let mut client = ControlClient::new();
    client.connect(syiot::remote::Transport::FramedTcp, "127.0.0.1:12000")?;

    let data = magicbox::State {
        joystick_x: 80,
        joystick_y: -40,
        rot_z: -10,
        switch: 1
    };

    loop {
        client.send(&data)?;
        std::thread::sleep(core::time::Duration::from_millis(1000));
    }
}