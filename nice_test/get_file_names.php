<?php
    $dir = $_GET['dir'];
    $files = array_diff(scandir($dir), array('..', '.'));

    header('Content-type: application/json');
    echo json_encode(array('files' => $files));
?>
