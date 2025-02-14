import 'package:flutter/material.dart';
import 'package:responsive_dashboard/utils/app_styles.dart';

class CustomButton extends StatelessWidget {
  const CustomButton({super.key, required this.text, this.isMain = true});
  final String text;
  final bool isMain;
  @override
  Widget build(BuildContext context) {
    return SizedBox(
      height: 64,
      child: ElevatedButton(
        style: ElevatedButton.styleFrom(
          shape: RoundedRectangleBorder(
            borderRadius: BorderRadius.circular(12),
          ),
          padding: const EdgeInsets.all(20),
          elevation: 0,
          backgroundColor:
              isMain ? const Color(0xFF4DB7F2) : Colors.transparent,
        ),
        onPressed: () {},
        child: Text(
          text,
          style: AppStyles.styleSemiBold18(context).copyWith(
            color: isMain ? Colors.white : const Color(0xFF4DB7F2),
          ),
        ),
      ),
    );
  }
}
