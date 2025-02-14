import 'package:flutter/material.dart';
import 'package:responsive_dashboard/views/widgets/custom_background_container.dart';
import 'package:responsive_dashboard/views/widgets/latest_transactions.dart';
import 'package:responsive_dashboard/views/widgets/quick_invoice_form.dart';
import 'package:responsive_dashboard/views/widgets/quick_invoice_header.dart';

class QuickInvoice extends StatelessWidget {
  const QuickInvoice({super.key});

  @override
  Widget build(BuildContext context) {
    return const CustomBackGroundContainer(
      padding: 24,
      child: Column(
        mainAxisAlignment: MainAxisAlignment.start,
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          QuickInvoiceHeader(),
          SizedBox(
            height: 24,
          ),
          LatestTransactions(),
          Divider(
            height: 48,
            color: Color(0xFF4DB7F2),
          ),
          QuickInvoiceForm(),
          
        ],
      ),
    );
  }
}
