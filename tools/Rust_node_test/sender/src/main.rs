// 发送节点1
use dora_node_api::{self, dora_core::config::DataId, DoraNode, Event, IntoArrow};
use eyre::Result;

fn main() -> Result<()> {
    println!("hello from node1");

    let output = DataId::from("data".to_owned());

    let (mut node, mut events) = DoraNode::init_from_env()?;
    

    for i in 0..100 {
        let i: u64 = i;
        let event = match events.recv() {
            Some(input) => input,
            None => break,
        };

        match event {
            Event::Input {
                id,
                metadata,
                data: _,
            } => match id.as_str() {
                "tick" => {
                    println!("tick {i}, sending {i} from node1");
                    node.send_output(output.clone(), metadata.parameters, i.into_arrow())?;
                }
                other => eprintln!("Ignoring unexpected input `{other}`"),
            },
            Event::Stop => {
                println!("Received manual stop");
                break;
            }
            other => eprintln!("Received unexpected input: {other:?}"),
        }
    }

    Ok(())
}
