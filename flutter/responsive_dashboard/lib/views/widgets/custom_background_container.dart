import 'package:flutter/material.dart';

class CustomBackGroundContainer extends StatelessWidget {
  const CustomBackGroundContainer({
    super.key,
    required this.child,
    required this.padding,
  });
  final Widget child;
  final double padding;
  @override
  Widget build(BuildContext context) {
    return Container(
        padding: EdgeInsets.all(padding),
        decoration: ShapeDecoration(
          color: Colors.white,
          shape: RoundedRectangleBorder(
            borderRadius: BorderRadius.circular(12),
          ),
        ),
        child: child);
  }
}