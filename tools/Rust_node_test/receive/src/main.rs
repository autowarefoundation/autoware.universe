use dora_node_api::{self, DoraNode, Event};
use eyre::Context;

fn main() -> eyre::Result<()> {
    println!("hello from receiver");

    let (_node, mut events) = DoraNode::init_from_env()?;

    let mut received_data_node1 = vec![false; 10000];
    let mut received_data_node2 = vec![false; 10000];

    while let Some(event) = events.recv() {
        match event {
            Event::Input { id, metadata: _, data, } => {
                match id.as_str() {
                    "data" => {
                        let value = u64::try_from(&data).context("unexpected data type")?;
                        if value < 100 {
                            received_data_node1[value as usize] = true;
                        }

                        let output = format!(
                            "operator received random value {value:#x} from node1" 
                        );
                        println!("{output}");
                    }
                    "data2" => {
                        let value = u64::try_from(&data).context("unexpected data type")?;
                        if value < 100 {
                            received_data_node2[value as usize] = true;
                        }

                        let output = format!(
                            "operator received random value {value:#x} from node2"
                        );
                        println!("{output}");
                    }
                    other => eprintln!("ignoring unexpected input {other}"),
                }
            }
            other => {
                println!("received unknown event {other:?}");
            }
        }
    }

    for i in 0..100 {
        if received_data_node1[i] {
            println!("Received value {i} from node1");
        } else {
            println!("Did not receive value {i} from node1");
        }

        if received_data_node2[i] {
            println!("Received value {i} from node2");
        } else {
            println!("Did not receive value {i} from node2");
        }
    }

    Ok(())
}
