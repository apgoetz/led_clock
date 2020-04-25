use app::goertzel::goertzel;

#[test]

// result of goerzel for pure DC signal is length of DFT 2
fn test_dc_values() {
    const N : usize = 20;
    let x : [f32;N] = [1.0;N];
    assert_eq!((N*N) as f32, goertzel(&x,0));
}
